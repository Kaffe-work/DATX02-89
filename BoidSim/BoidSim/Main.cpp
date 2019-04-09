#include "glad/glad.h"
#include <GLFW/glfw3.h>
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>
#include <map>
#include "Shader.h"
#include <list>
#include "boid.h"
#include "spatial_hash.hpp"
#include <algorithm>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include <iostream>

#include <ft2build.h>
#include FT_FREETYPE_H  

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow *window);
double xpos, ypos; // cursor position

// setup
const unsigned int screenWidth = 1280, screenHeight = 720;

glm::vec3 cameraDir(1.0f, 1.0f, 200.0f);
glm::vec3 cameraPos(1.0f, 1.0f, -200.0f);
double yaw = 1.6f, pitch = 0.0f;

// How many boids on screen
const int nrBoids = 1000;
std::vector<Boid> boids;

// Boid attributes
const float MAX_SPEED = 30.0f;
const float MIN_SPEED = 20.0f;
const float MAX_NOISE = 2.0f;
bool repellLine = false;

// Time, used to print performance
double lastTime = glfwGetTime();
int nrFrames = 0;

// Vertex Array Object, Vertex/Element Buffer Objects, texture (can be reused)
unsigned int VAO, VBO, EBO, tex1, tex2;
int nrTextures = 0;

// Score
int score = 0;


// Create a struct for characters
struct Character {
	GLuint     TextureID;  // ID handle of the glyph texture
	glm::ivec2 Size;       // Size of glyph
	glm::ivec2 Bearing;    // Offset from baseline to left/top of glyph
	GLuint     Advance;    // Offset to advance to next glyph
};

// A hashmap storing characters so we only have to instantiate each char once
std::map<GLchar, Character> Characters;

void loadChars() {
	FT_Library ft;
	if (FT_Init_FreeType(&ft))
		std::cout << "ERROR::FREETYPE: Could not init FreeType Library" << std::endl;

	FT_Face face;
	if (FT_New_Face(ft, "fonts/Anton.ttf", 0, &face))
		std::cout << "ERROR::FREETYPE: Failed to load font" << std::endl;

	FT_Set_Pixel_Sizes(face, 0, 48);

	glPixelStorei(GL_UNPACK_ALIGNMENT, 1); // Disable byte-alignment restriction

	for (GLubyte c = 0; c < 128; c++)
	{
		// Load character glyph 
		if (FT_Load_Char(face, c, FT_LOAD_RENDER))
		{
			std::cout << "ERROR::FREETYTPE: Failed to load Glyph" << std::endl;
			continue;
		}
		// Generate texture
		GLuint texture;
		glGenTextures(1, &texture);
		glBindTexture(GL_TEXTURE_2D, texture);
		glTexImage2D(
			GL_TEXTURE_2D,
			0,
			GL_RED,
			face->glyph->bitmap.width,
			face->glyph->bitmap.rows,
			0,
			GL_RED,
			GL_UNSIGNED_BYTE,
			face->glyph->bitmap.buffer
		);
		// Set texture options
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		// Now store character for later use
		Character character = {
			texture,
			glm::ivec2(face->glyph->bitmap.width, face->glyph->bitmap.rows),
			glm::ivec2(face->glyph->bitmap_left, face->glyph->bitmap_top),
			face->glyph->advance.x
		};
		Characters.insert(std::pair<GLchar, Character>(c, character));
	}
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

	FT_Done_Face(face);
	FT_Done_FreeType(ft);
}

// Reference: http://www.opengl-tutorial.org/miscellaneous/an-fps-counter/
void printPerformance() {
	// Print if 1 sec has passed since last time
	double currentTime = glfwGetTime();
	nrFrames++;
	if (currentTime - lastTime >= 1.0) {
		// print number of agents only once
		// print data and reset
		std::cout << "avg draw time: " << 1000 / double(nrFrames) << "ms, fps: " << nrFrames << std::endl;
		nrFrames = 0;
		lastTime += 1.0;
	}
}

// If e.g. percentage = 1 => vec3(0,0,0) will be returned with 99% probability
glm::vec3 getRandomVectorWithChance(int percentage) {
	bool maybe = percentage == 0 ? false : rand() % (100/percentage) == 0;
	return glm::vec3(maybe ? rand() % 121 - 60, rand() % 121 - 60, rand() % 21 - 10 : 0, 0, 0);
}

void updateBoids(Boid & b) { // Flocking rules are implemented here

	/*Alignment = Velocity Matching*/
	//Sum the velocities of the neighbours and this boid and average them.

	/*Separation = Collision Avoidance*/
	//Sum the vectors from all neighbours to this boid.  

	/*Cohesion - Flock Centering*/
	//Sum the positions of the neighbours and average them, then subtract this boids position

	glm::vec3 alignment = b.velocity;
	glm::vec3 separation = glm::vec3(0.0);
	glm::vec3 cohesion = glm::vec3(0.0);
	glm::vec3 repellation = glm::vec3(0.0);

	std::vector<Boid*> nb = getNeighbours(b);
	if (std::size(nb) == 0) { 
		b.velocity *= glm::clamp(length(b.velocity), MIN_SPEED, MAX_SPEED);
		return;	
	}
	for (Boid* n : nb) {
		Boid neighbour = *n;
		alignment += neighbour.velocity * 4.0f/distance(b.position, neighbour.position);
		separation += (b.position - neighbour.position) * 1.0f/(float)(pow(distance(b.position, neighbour.position),2) + 0.0001); // + 0.0001 is for avoiding divide by zero
		cohesion += neighbour.position;
	}
	alignment = alignment * (1.0f / (std::size(nb) + 1));
	cohesion = cohesion * (1.0f / std::size(nb)) - b.position;
	separation = separation * (1.0f / std::size(nb));

	/*Repellation - Escape*/
	//Inverse square function of distance between point of repellation and a boid, from the KTH paper about Sheep and a predator.
	//Point of repellation: A + dot(AP,AB) / dot(AB,AB) * AB
	
	if (repellLine) {
		glm::vec3 point = cameraPos + dot(b.position - cameraPos, cameraDir) / dot(cameraDir, cameraDir) * (cameraDir);
		repellation = normalize(b.position - point)*(1.0f / pow(distance(b.position, point) / 5.0f + 2.0f, 2));
	}
	
	/*Update Velocity*/
	glm::vec3 newVel = alignment + 50.0f*separation + 0.9f*cohesion + 100.0f*repellation + MAX_NOISE * getRandomVectorWithChance(0.5f);
	float speed = glm::clamp(length(newVel), MIN_SPEED, MAX_SPEED); // limit speed

	/*Update Velocity*/
	b.velocity = speed*glm::normalize(newVel);
}

void renderCrosshair() {
	float vertices[] = {
		// position hand     // texture coords
	   -0.05f*screenHeight / screenWidth,-0.05f,  0.0f,  0.0f, 0.0f,
		0.05f*screenHeight / screenWidth,-0.05f,  0.0f,  1.0f, 0.0f,
		0.05f*screenHeight / screenWidth, 0.05f,  0.0f,  1.0f, 1.0f,
	   -0.05f*screenHeight / screenWidth, 0.05f,  0.0f,  0.0f, 1.0f
	};

	unsigned int indices[] = {
		0, 1, 3,   // first triangle
		1, 2, 3    // second triangle
	};

	glBindVertexArray(VAO);

	// For the indices
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

	// position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	// texture coordinates
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(2);

	glBindTexture(GL_TEXTURE_2D, tex2);
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
}

void renderLaser() {
	float vertices[] = {
		// positions         // color
		0.6f,  -0.2f, 0.0f,  1.0f, 0.0f, 0.0f,
		0.55f, -0.2f, 0.0f,  1.0f, 0.0f, 0.0f,
		0.01f,  0.0f, 0.0f,  1.0f, 0.0f, 0.0f,
		0.0f,   0.0f, 0.0f,  1.0f, 0.0f, 0.0f
	};

	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

	// position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	// color attribute
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);

	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

	// unbind buffer and vertex array
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

}

void renderWeapon() {
	float vertices[] = {
		// position hand     // texture coords
		0.1f,  0.0f,  0.0f,  0.0f, 0.0f,
		1.2f,  0.0f,  0.0f,	 1.0f, 0.0f,
	    1.2f, -1.0f,  0.0f,  1.0f, 1.0f,
		0.1f, -1.0f,  0.0f,  0.0f, 1.0f
	};

	unsigned int indices[] = {
		0, 1, 3,   // first triangle
		1, 2, 3    // second triangle
	};

	glBindVertexArray(VAO);
	
	// For the indices
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

	// position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	
	// texture coordinates
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(2);

	glBindTexture(GL_TEXTURE_2D, tex1);
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
}

void renderText(Shader &s, std::string text, GLfloat x, GLfloat y, GLfloat scale, glm::vec3 color)
{
	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 6 * 4, NULL, GL_DYNAMIC_DRAW);

	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), 0);
	glEnableVertexAttribArray(0);

	// Activate corresponding render state	
	s.use();
	s.setVec3("textColor", color);
	s.setMatrix("projection", glm::ortho(0.0f, (float)screenWidth, 0.0f, (float)screenHeight));

	// Iterate through all characters
	std::string::const_iterator c;
	for (c = text.begin(); c != text.end(); c++)
	{
		Character ch = Characters[*c];

		GLfloat xpos = x + ch.Bearing.x * scale;
		GLfloat ypos = y - (ch.Size.y - ch.Bearing.y) * scale;

		GLfloat w = ch.Size.x * scale;
		GLfloat h = ch.Size.y * scale;
		// Update VBO for each character
		GLfloat vertices[6][4] = {
			{ xpos,     ypos + h,   0.0, 0.0 },
			{ xpos,     ypos,       0.0, 1.0 },
			{ xpos + w, ypos,       1.0, 1.0 },

			{ xpos,     ypos + h,   0.0, 0.0 },
			{ xpos + w, ypos,       1.0, 1.0 },
			{ xpos + w, ypos + h,   1.0, 0.0 }
		};
		// Render glyph texture over quad
		glBindTexture(GL_TEXTURE_2D, ch.TextureID);
		// Update content of VBO memory
		glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices);
		// Render quad
		glDrawArrays(GL_TRIANGLES, 0, 6);
		// Now advance cursors for next glyph (note that advance is number of 1/64 pixels)
		x += (ch.Advance >> 6) * scale; // Bitshift by 6 to get value in pixels (2^6 = 64)
	}
}

// Used to create textures from images (e.g. rifle and crosshair)
void createTexture(unsigned int &ref, const char* path) {
	// Bind texture for gun
	glBindTexture(GL_TEXTURE_2D, ref);
	// No repeat, use actual size of picture
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	// set texture filtering parameters
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	// load image, create texture
	int width, height, nrChannels;
	unsigned char *data = stbi_load(path, &width, &height, &nrChannels, 0);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);

	stbi_image_free(data);
}

int main()
{
	// glfw: initialize and configure
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // Needed for OS X, and possibly Linux

	// glfw: window creation
	GLFWwindow* window = glfwCreateWindow(screenWidth, screenHeight, "BoidSim", NULL, NULL);
	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

	// glad: load all OpenGL function pointers
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		return -1;
	}

	// Load characters for text to hashmap
	loadChars();

	// create the boids (allocate space and randomize position/velocity by calling constructor)
	for (int i = 0; i < nrBoids; ++i)
		boids.push_back(Boid());
	
	// one vector for each vertex
	glm::vec3 p1(-1.0f, -1.0f, 0.0f);
	glm::vec3 p2(0.0f, 1.0f, 0.0f);
	glm::vec3 p3(1.0f, -1.0f, 0.0f);

	// generate things
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);
	glGenTextures(1, &tex1);
	glGenTextures(1, &tex2);
		
	// Build and compile shaders
	Shader shader("vert.shader", "frag.shader");
	Shader guiShader("gui.shader", "frag.shader");
	Shader guiShader2("gui.shader", "gui.frag");
	Shader textShader("text.vert", "text.frag");

	// Create textures from images
	createTexture(tex1, "rifle.png");
	createTexture(tex2, "crosshair.png");

	// Enable alpha channel (transparancy) for textures
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// use (bind) the shader 1 so that we can attach matrices
	shader.use();

	// instantiate transformation matrices
	glm::mat4 projection, view, model;
	// projection will always be the same: define FOV, aspect ratio and view frustum (near & far plane)
	projection = glm::perspective(glm::radians(45.0f), (float)screenWidth / screenHeight, 0.1f, 1000.0f);
	// set projection matrix as uniform (attach to bound shader)
	shader.setMatrix("projection", projection);

	// instantiate array for boids
	glm::vec3 renderBoids[nrBoids*3];

	// render loop
	// -----------
	while (!glfwWindowShouldClose(window))
	{
		// Need to choose shader since we now have 2
		shader.use();
		// print performance to console
		printPerformance();
		// if got input, processed here
		processInput(window);

		// update camera direction, rotation
		cameraDir = glm::vec3(cos(pitch)*cos(yaw), sin(-pitch), cos(pitch)*sin(yaw));
		normalize(cameraDir);
		// calculate view-matrix based on cameraDir and cameraPos
		view = glm::lookAt(cameraPos, cameraPos + cameraDir, glm::vec3(0.0f, 1.0f, 0.0f));
		// clear whatever was on screen last frame

		glClearColor(0.90f, 0.95f, 0.96f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);

		// Put all boids in the hash table so we can use it in the next loop
		for (Boid& b : boids){
			putInHashTable(b);
		}

		int i = 0;

		// move each boid to current pos, update pos given velocity
		for (Boid& b : boids)
		{
			// update boid velocity
			updateBoids(b);

			// move the boid in its facing direction
			b.position += 0.01f * b.velocity;

			// create model matrix from agent position
			glm::mat4 model = glm::mat4(1.0f);
			model = glm::translate(model, b.position);
			glm::vec3 v = glm::vec3(b.velocity.z, 0, -b.velocity.x);
			float angle = acos(b.velocity.y / glm::length(b.velocity));
			model = glm::rotate(model, angle, v);

			// transform each vertex and add them to array
			renderBoids[i++] = view * model * glm::vec4(p1, 1.0f);
			renderBoids[i++] = view * model * glm::vec4(p2, 1.0f);
			renderBoids[i++] = view * model * glm::vec4(p3, 1.0f);
		}
		clearHashTable();

		// bind vertex array
		glBindVertexArray(VAO);
		// bind buffer object and boid array
		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, nrBoids * sizeof(glm::vec3) * 3, &renderBoids[0], GL_STATIC_DRAW);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(0);

		// Draw 3 * nrBoids vertices
		glDrawArrays(GL_TRIANGLES, 0, nrBoids * 3);

		// unbind buffer and vertex array
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);

		guiShader.use();
		int state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
		state = glfwGetKey(window, GLFW_KEY_SPACE);
		if (state == GLFW_PRESS) {
			renderLaser();
		}

		guiShader2.use();
		renderWeapon();
		renderCrosshair();

		renderText(textShader, std::to_string(score), 25.0f, 25.0f, 1.0f, glm::vec3(0.5, 0.8f, 0.2f));

		// glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	// optional: de-allocate all resources once they've outlived their purpose:
	glDeleteVertexArrays(1, &VAO);
	glDeleteBuffers(1, &VBO);

	// glfw: terminate, clearing all previously allocated GLFW resources.
	glfwTerminate();
	return 0;
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
void processInput(GLFWwindow *window)
{
	double xpos_old = xpos;
	double ypos_old = ypos;
	glfwGetCursorPos(window, &xpos, &ypos);
	double deltaX = xpos - xpos_old;
	double deltaY = ypos - ypos_old;

	// If left mouse click is depressed, modify yaw and pitch
	int state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
	if (state != GLFW_PRESS) {
		yaw += deltaX * 0.002;
		pitch = fmin(pitch + deltaY * 0.002, 0.3f); // max 89 grader
	}

	state = glfwGetKey(window, GLFW_KEY_W);
	if (state == GLFW_PRESS) {
		cameraPos += cameraDir * 3.0f;
	}
	state = glfwGetKey(window, GLFW_KEY_S);
	if (state == GLFW_PRESS) {
		cameraPos -= cameraDir * 3.0f;
	}
	state = glfwGetKey(window, GLFW_KEY_A);
	if (state == GLFW_PRESS) {
		cameraPos -= cross(cameraDir, glm::vec3(0.0f, 1.0f, 0.0f)) * 3.0f;
	}
	state = glfwGetKey(window, GLFW_KEY_D);
	if (state == GLFW_PRESS) {
		cameraPos += cross(cameraDir, glm::vec3(0.0f, 1.0f, 0.0f)) * 3.0f;
	}
	state = glfwGetKey(window, GLFW_KEY_SPACE);
	if (state == GLFW_PRESS) {
		repellLine = true;
	}
	else {
		repellLine = false;
	}

	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	// make sure the viewport matches the new window dimensions; note that width and 
	// height will be significantly larger than specified on retina displays.
	glViewport(0, 0, width, height);
}