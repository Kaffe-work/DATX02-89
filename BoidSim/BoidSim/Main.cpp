#include "levelfactory.h"
#include "glad/glad.h"
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>
#include "Shader.h"
#include <list>
#include "boid.h"
#include "obstaclepoint.h"
#include "obstacleplane.h"
#include "spatial_hash.hpp"
#include <algorithm>
#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include <iostream>

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow *window);
double xpos, ypos; // cursor position

bool cameraReset = true;
				   
// Number of boids, with nrPredators as predators. 
const int nrBoids = 2000;

// setup
const unsigned int screenWidth = 1280, screenHeight = 720;

// camera settings
glm::vec3 cameraDir(1.0f, 1.0f, 200.0f);
glm::vec3 cameraPos(1.0f, 1.0f, -200.0f);
double yaw = 1.6f, pitch = 0.0f;

// Vertex Array Object, Vertex/Element Buffer Objects, texture (can be reused)
unsigned int VAO, VBO, EBO, tex1, tex2;

// For ImGui
bool show_demo_window = true;
bool show_another_window = false;
ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

glm::vec3 getSteeringPredator(Boid & b) {
	if (!b.isAlive) {
		return glm::vec3(0.0);
	}

	glm::vec3 alignment = glm::vec3(0.0);
	glm::vec3 separation = glm::vec3(0.0);
	glm::vec3 cohesion = glm::vec3(0.0);
	glm::vec3 planeforce = glm::vec3(0.0);
	glm::vec3 lineforce = glm::vec3(0.0);
	glm::vec3 hunt = glm::vec3(0.0);
	std::vector<Boid*> nb = getNeighbours(b);
	std::vector<Boid> prey;
	std::vector<Boid> predators;

	for (Boid* n : nb) {
		Boid neighbour = *n;
		if (neighbour.isAlive) {
			if (neighbour.isPredator) {
				predators.push_back(neighbour);
			}
			else {
				prey.push_back(neighbour);
			}
		}
	}

	for (Boid n : predators) {
		alignment += n.velocity;
		cohesion += n.position;
		separation += normalize(b.position - n.position) * SEPARATION_SOFTNESS / distance(b.position, n.position);
	}

	if (std::size(predators) > 0) {
		alignment = normalize(alignment * (1.0f / std::size(predators)) - b.velocity);
		cohesion = normalize(cohesion * (1.0f / std::size(predators)) - b.position - b.velocity);
		separation = normalize(separation * (1.0f / std::size(predators)) - b.velocity);
	}

	//Avoid walls
	for (ObstaclePlane o : walls) {
		glm::vec3 v = b.position - o.point;
		float distance = PLANE_SOFTNESS / glm::dot(v, o.normal);
		planeforce += normalize(o.normal)*distance - b.velocity;
	}

	//Avoid player controlled line
	if (isLaserActive) {
		glm::vec3 point = cameraPos + dot(b.position - cameraPos, cameraDir) / dot(cameraDir, cameraDir) * (cameraDir);
		if (isLaserLethal && distance(b.position, point) < DEATH_DISTANCE) {
			b.isAlive = false;
			scorePositive++;
			return glm::vec3(0.0);
		}
		lineforce = (isLaserRepellant ? 1.0f : -1.0f) * normalize(b.position - point) * LASER_SOFTNESS / (distance(b.position, point)) - b.velocity;
	}

	//Hunt
	if (size(prey) > 0) {
		Boid closestPrey = prey[0];
		for (Boid o : prey) {
			if (distance(closestPrey.position, b.position) > distance(o.position, b.position)) {
				closestPrey = o;
			}
		}
		hunt = normalize(closestPrey.position - b.position - b.velocity);
	}


	glm::vec3 steering = alignment + cohesion + 2.0f*separation + 10.0f*planeforce + 50.0f*hunt + lineforce;
	if (!is3D) { steering = glm::vec3(steering.x, steering.y, 0); }

	// Limit acceleration
	float magnitude = glm::clamp(glm::length(steering), 0.0f, MAX_ACCELERATION_PREDATOR);
	return magnitude * glm::normalize(steering);
}


glm::vec3 getSteeringPrey(Boid & b) { // Flocking rules are implemented here

	if (!b.isAlive) {
		return glm::vec3(0.0);
	}

	glm::vec3 alignment = glm::vec3(0.0);
	glm::vec3 separation = glm::vec3(0.0);
	glm::vec3 cohesion = glm::vec3(0.0);
	glm::vec3 lineforce = glm::vec3(0.0);
	glm::vec3 planeforce = glm::vec3(0.0);
	glm::vec3 pointforce = glm::vec3(0.0);
	glm::vec3 flee = glm::vec3(0.0);

	std::vector<Boid*> nb = getNeighbours(b);
	std::vector<Boid> prey;
	std::vector<Boid> predators;

	for (Boid* n : nb) {
		Boid neighbour = *n;
		if (neighbour.isAlive) {
			if (neighbour.isPredator) {
				predators.push_back(neighbour);
			}
			else {
				prey.push_back(neighbour);
			}
		}
	}

	//Flocking rules
	for (Boid n : prey) {
		alignment += n.velocity;
		cohesion += n.position;
		separation += normalize(b.position - n.position) / distance(b.position, n.position);
	}

	if (std::size(prey) > 0) {
		alignment = normalize(alignment * (1.0f / std::size(prey)) - b.velocity);
		cohesion = normalize(cohesion * (1.0f / std::size(prey)) - b.position - b.velocity);
		separation = normalize(separation * (1.0f / std::size(prey)) - b.velocity);
	}

	//Avoid planes
	for (ObstaclePlane o : walls) {
		glm::vec3 v = b.position - o.point;
		float distance = PLANE_SOFTNESS / glm::dot(v, o.normal);
		planeforce += normalize(o.normal)*distance - b.velocity;

	}

	//Avoid/steer towards an obstaclepoint, and "Exit (die)"
	for (ObstaclePoint f : objects) {
		if (f.lethality && distance(b.position, f.position) < DEATH_DISTANCE) {
			b.isAlive = false;
			scoreNegative++;
			return glm::vec3(0.0);
		}
		pointforce += (f.attractive ? -1.0f : 1.0f) * normalize(b.position - f.position) * 2.0f / distance(b.position, f.position);
	}
	if (std::size(objects) > 0) {
		pointforce = normalize(pointforce * (1.0f / std::size(objects)) - b.velocity);
	}

	//Avoid player controlled line
	if (isLaserActive) {
		glm::vec3 point = cameraPos + dot(b.position - cameraPos, cameraDir) / dot(cameraDir, cameraDir) * (cameraDir);
		if (isLaserLethal && distance(b.position, point) < DEATH_DISTANCE) {
			b.isAlive = false;
			scorePositive++;
			return glm::vec3(0.0);
		}
		lineforce = (isLaserRepellant ? 1.0f : -1.0f) * normalize(b.position - point) * LASER_SOFTNESS / (distance(b.position, point)) - b.velocity;
	}


	//Flee predators and check death
	if (size(predators) > 0) {
		for (Boid n : predators) {
			flee += 1.0f/n.position;
			if (distance(n.position, b.position) < DEATH_DISTANCE) {
				b.isAlive = false;
				scorePositive++;
				return glm::vec3(0.0);
			}
		}
		flee = - normalize(flee * (1.0f / std::size(predators)) - b.position - b.velocity);
	}
	
	glm::vec3 steering = alignment + cohesion + 1.5f*separation + 10.0f*planeforce + 10.0f*pointforce + lineforce + flee;
	if (!is3D) { steering = glm::vec3(steering.x, steering.y, 0); }

	// Limit acceleration
	float magnitude = glm::clamp(glm::length(steering), 0.0f, MAX_ACCELERATION); 
	return magnitude*glm::normalize(steering);

}

unsigned int loadCubemap(std::vector<std::string> faces)
{
	unsigned int textureID;
	glGenTextures(1, &textureID);
	glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);

	int width, height, nrChannels;
	for (unsigned int i = 0; i < faces.size(); i++)
	{
		unsigned char *data = stbi_load(faces[i].c_str(), &width, &height, &nrChannels, 0);
		if (data)
		{
			glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i,
				0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data
			);
			stbi_image_free(data);
		}
		else
		{
			std::cout << "Cubemap texture failed to load at path: " << faces[i] << std::endl;
			stbi_image_free(data);
		}
	}
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

	return textureID;
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
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	stbi_image_free(data);
}

void createImGuiWindow()
{
	static float f = 0.0f;
	static int counter = 0;

	ImGui::Begin("Hello, world!");                          // Create a window called "Hello, world!" and append into it.

	/*ImGui::Text("This is some useful text.");               // Display some text (you can use a format strings too)
	ImGui::Checkbox("Demo Window", &show_demo_window);      // Edit bools storing our window open/close state
	ImGui::Checkbox("Another Window", &show_another_window);

	ImGui::SliderFloat("float", &f, 0.0f, 1.0f);            // Edit 1 float using a slider from 0.0f to 1.0f
	ImGui::ColorEdit3("clear color", (float*)&clear_color); // Edit 3 floats representing a color

	if (ImGui::Button("Button"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
		counter++;
	ImGui::SameLine();*/
	ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
	ImGui::Text("Score = %d", scorePositive - scoreNegative);

	if (scorePositive + scoreNegative == nrBoids) {
		ImGui::Text("Game over!");
	}
	ImGui::End();
}

void resetCamera() {
	cameraDir = glm::vec3(1.0f, 1.0f, 200.0f);
	cameraPos = glm::vec3(1.0f, 1.0f, -200.0f);
	yaw = 1.6f;
	pitch = 0.0f;
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
	GLFWwindow* window = glfwCreateWindow(screenWidth, screenHeight, "BoidSim", glfwGetPrimaryMonitor(), NULL);
	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	
	// GLFW catches the cursor
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	// glad: load all OpenGL function pointers
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		return -1;
	}

	// load background (cubemap)
	std::vector<std::string> faces
	{
		    "skybox/right.jpg",
			"skybox/left.jpg",
			"skybox/top.jpg",
			"skybox/bottom.jpg",
			"skybox/front.jpg",
			"skybox/back.jpg"
	};
	unsigned int cubemapTexture = loadCubemap(faces);

	float skyboxVertices[] = {
		// positions          
		-1.0f,  1.0f, -1.0f,
		-1.0f, -1.0f, -1.0f,
		1.0f, -1.0f, -1.0f,
		1.0f, -1.0f, -1.0f,
		1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f,

		-1.0f, -1.0f,  1.0f,
		-1.0f, -1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f,  1.0f,
		-1.0f, -1.0f,  1.0f,

		1.0f, -1.0f, -1.0f,
		1.0f, -1.0f,  1.0f,
		1.0f,  1.0f,  1.0f,
		1.0f,  1.0f,  1.0f,
		1.0f,  1.0f, -1.0f,
		1.0f, -1.0f, -1.0f,

		-1.0f, -1.0f,  1.0f,
		-1.0f,  1.0f,  1.0f,
		1.0f,  1.0f,  1.0f,
		1.0f,  1.0f,  1.0f,
		1.0f, -1.0f,  1.0f,
		-1.0f, -1.0f,  1.0f,

		-1.0f,  1.0f, -1.0f,
		1.0f,  1.0f, -1.0f,
		1.0f,  1.0f,  1.0f,
		1.0f,  1.0f,  1.0f,
		-1.0f,  1.0f,  1.0f,
		-1.0f,  1.0f, -1.0f,

		-1.0f, -1.0f, -1.0f,
		-1.0f, -1.0f,  1.0f,
		1.0f, -1.0f, -1.0f,
		1.0f, -1.0f, -1.0f,
		-1.0f, -1.0f,  1.0f,
		1.0f, -1.0f,  1.0f
	};


	//Initialise boids, walls, objects
	createLevel(nrBoids);

	

	// one vector for each vertex
	glm::vec3 p1(-1.0f, -1.0f, 0.0f);
	glm::vec3 p2(0.0f, 1.0f, 0.0f);
	glm::vec3 p3(1.0f, -1.0f, 0.0f);

	// generate things
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);

	// setup skybox VAO and VBO data
	unsigned int skyboxVAO, skyboxVBO;
	glGenVertexArrays(1, &skyboxVAO);
	glGenBuffers(1, &skyboxVBO);
	glBindVertexArray(skyboxVAO);
	glBindBuffer(GL_ARRAY_BUFFER, skyboxVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(skyboxVertices), &skyboxVertices, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

	// use the shader created earlier so we can attach matrices
	glGenBuffers(1, &EBO);
	glGenTextures(1, &tex1);
		
	// Build and compile shaders
	Shader shader("simple.vert", "simple.frag");
	Shader skybox("cube.vert", "cube.frag");
	Shader laserShader("gui.vert", "simple.frag");
	Shader guiShader("gui.vert", "gui.frag");

	// Create textures
	createTexture(tex1, "rifle.png");
	createTexture(tex2, "crosshair.png");

	// use (bind) the shader 1 so that we can attach matrices
	shader.use();

	// instantiate transformation matrices
	glm::mat4 projection, view, model;
	// projection will always be the same: define FOV, aspect ratio and view frustum (near & far plane)
	projection = glm::perspective(glm::radians(45.0f), (float)screenWidth / screenHeight, 0.1f, 1000.0f);
	// set projection matrix as uniform (attach to bound shader)
	shader.setMatrix("projection", projection);

	// skybox (background) uses the same projection matrix
	skybox.use();
	skybox.setMatrix("projection", projection);

	// instantiate array for boids
	glm::vec3 renderBoids[nrBoids*3*2]; // Each boid has three points and RGB color

	// Dear ImGui setup
	ImGui::CreateContext();
	ImGui::StyleColorsDark();
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init("#version 330"); // glsl version


	// render loop
	// -----------
	while (!glfwWindowShouldClose(window))
	{
		// Need to choose shader since we now have 2
		shader.use();
		// if got input, processed here
		processInput(window);

		// Setup frame for ImGui
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		// update camera direction, rotation
		cameraDir = glm::vec3(cos(pitch)*cos(yaw), sin(-pitch), cos(pitch)*sin(yaw));
		normalize(cameraDir);
		// calculate view-matrix based on cameraDir and cameraPos
		view = glm::lookAt(cameraPos, cameraPos + cameraDir, glm::vec3(0.0f, 1.0f, 0.0f));
		// clear whatever was on screen last frame

		glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		if (cameraReset) {
			resetCamera();
			cameraReset = false;
		}

		// Put all boids in the hash table so we can use it in the next loop
		for (Boid& b : boids){
			putInHashTable(b);
		}

		for (int i = 0; i < nrBoids; i++)
			{
				// Calculate new velocities for each boid, update pos given velocity
				if (boids[i].isPredator){
					boids[i].velocity += getSteeringPredator(boids.at(i));
					boids[i].velocity = normalize(boids[i].velocity)*MAX_SPEED_PREDATOR;
				}
				else {
					boids[i].velocity += getSteeringPrey(boids.at(i));
					boids[i].velocity = normalize(boids[i].velocity)*MAX_SPEED;
				}
				
				boids[i].position += boids[i].velocity;
				

				// create model matrix from agent position
				glm::mat4 model = glm::mat4(1.0f);
				model = glm::translate(model, boids[i].position);
				glm::vec3 v = glm::vec3(boids[i].velocity.z, 0, -boids[i].velocity.x);
				float angle = acos(boids[i].velocity.y / glm::length(boids[i].velocity));
				model = glm::rotate(model, angle, v);

				// transform each vertex and add them to array
					renderBoids[i * 6] = view * model * glm::vec4(p1, 1.0f);
					renderBoids[i * 6 + 1] = glm::vec3(1.0f) * (float)!boids[i].isPredator + glm::vec3(3.0f)*(float)!boids[i].isAlive; // color vertex 1
					renderBoids[i * 6 + 2] = view * model * glm::vec4(p2, 1.0f);
					renderBoids[i * 6 + 3] = glm::vec3(1.0f) * (float)!boids[i].isPredator + glm::vec3(3.0f)*(float)!boids[i].isAlive; // color vertex 2
					renderBoids[i * 6 + 4] = view * model * glm::vec4(p3, 1.0f);
					renderBoids[i * 6 + 5] = glm::vec3(1.0f) * (float)!boids[i].isPredator + glm::vec3(3.0f)*(float)!boids[i].isAlive; // color vertex 3
			}

		clearHashTable();

		// draw skybox
		glDepthFunc(GL_LEQUAL);
		skybox.use();
		skybox.setMatrix("view", glm::mat4(glm::mat3(view)));
		// ... set view and projection matrix
		glBindVertexArray(skyboxVAO);
		glBindTexture(GL_TEXTURE_CUBE_MAP, cubemapTexture);
		glDrawArrays(GL_TRIANGLES, 0, 36);
		glDepthFunc(GL_LESS); // set depth function back to default

		shader.use();
		// bind vertex array
		glBindVertexArray(VAO);
		// bind buffer object and boid array
		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, nrBoids * sizeof(glm::vec3) * 3 * 2, &renderBoids[0], GL_STATIC_DRAW);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(0);

		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
		glEnableVertexAttribArray(1);

		// Draw 3 * nrBoids vertices
		glDrawArrays(GL_TRIANGLES, 0, nrBoids * 3);

		// unbind buffer and vertex array
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);


		if (isLaserActive) {
			laserShader.use();
			renderLaser();
		}

		guiShader.use();
		renderWeapon();
		renderCrosshair();

		// ImGui create/render window
		createImGuiWindow();
		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		// glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	// optional: de-allocate all resources once they've outlived their purpose:
	glDeleteVertexArrays(1, &VAO);
	glDeleteBuffers(1, &VBO);

	// terminate, clearing all previously allocated GLFW/ImGui resources.
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
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
	yaw += deltaX * 0.002;
	pitch = fmin(pitch + deltaY * 0.002, 0.3f); // max 89 grader


	int state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
	if (state == GLFW_PRESS) {
		isLaserActive = true;
	}else {
		isLaserActive = false;
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
	state = glfwGetKey(window, GLFW_KEY_1);
	if (state == GLFW_PRESS) {
		reset(nrBoids, 1);
		resetCamera();
	}
	state = glfwGetKey(window, GLFW_KEY_2);
	if (state == GLFW_PRESS) {
		reset(nrBoids, 2);
		resetCamera();
	}
	state = glfwGetKey(window, GLFW_KEY_3);
	if (state == GLFW_PRESS) {
		reset(nrBoids, 3);
		resetCamera();
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