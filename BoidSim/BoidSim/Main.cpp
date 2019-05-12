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

#include "tbb/parallel_for.h"
#include "tbb/task_scheduler_init.h"

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void processInput(GLFWwindow *window);

GLFWwindow* window;

bool cameraReset = true; 
const int nrBoids = 1000;
const unsigned int screenWidth = 1920, screenHeight = 1080;

// Camera variables
glm::vec3 cameraPos = glm::vec3(0.0f, 0.0f, 3.0f);
glm::vec3 cameraDir = glm::vec3(0.0f, 0.0f, 2.0f);
glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);

bool firstMouse = true;
float yaw = -90.0f;	// yaw is initialized to -90.0 degrees since a yaw of 0.0 results in a direction vector pointing to the right so we initially rotate a bit to the left.
float pitch = 0.0f;
float lastX = 800.0f / 2.0;
float lastY = 600.0 / 2.0;
float fov = 45.0f;

// timing
float deltaTime = 0.0f;	// time between current frame and last frame
float lastFrame = 0.0f;

// lighting
glm::vec3 lightPos(1.0f, 100.0f, -100.0f);
glm::vec3 lightColor(1.0f, 1.0f, 1.0f);

unsigned int VAO, VBO;
double timeElapsed, b4, avgTimeElapsed, fps; // benchmarking stuff
int frame;

glm::vec3 getSteering(Boid & b) { // Flocking rules are implemented here

	glm::vec3 alignment = glm::vec3(0.0);
	glm::vec3 separation = glm::vec3(0.0);
	glm::vec3 cohesion = glm::vec3(0.0);
	glm::vec3 planeforce = glm::vec3(0.0);
	glm::vec3 v;
	float dist;

	std::vector<Boid*> nb = getNeighbours(b);
	std::vector<Boid> prey;
	std::vector<Boid> predators;

	for (Boid *neighbour : nb) {
		Boid n = *neighbour;
		if (b.position == n.position || distance(b.position, n.position) < 15.0f) continue;

		alignment += n.velocity;
		cohesion += n.position;
		separation += normalize(b.position - n.position) / sqrt(distance(b.position, n.position));
	}

	for (ObstaclePlane o : walls) {
		v = b.position - o.point;
		dist = PLANE_SOFTNESS / glm::dot(v, o.normal);
		planeforce += normalize(o.normal) * dist - b.velocity;

	}
	
	glm::vec3 steering = 30.0f*alignment + cohesion + 1.5f*separation + 10.0f*planeforce;

	// Limit acceleration
	return glm::clamp(glm::length(steering), 0.0f, MAX_ACCELERATION)*glm::normalize(steering);

}

void createImGuiWindow()
{
	static float f = 0.0f;
	static int counter = 0;

	ImGui::Begin("Performance:");

	ImGui::Text("Application average %.3f ms/frame (%.1f FPS), timeElapsed: %f", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate, avgTimeElapsed);

	ImGui::End();
}

int initGLFW()
{
	// glfw: initialize and configure
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // Needed for OS X, and possibly Linux

														 // glfw: window creation
	window = glfwCreateWindow(screenWidth, screenHeight, "BoidSim", NULL/*glfwGetPrimaryMonitor()*/, NULL);
	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	glfwSetCursorPosCallback(window, mouse_callback);

	// GLFW catches the cursor
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	// glad: load all OpenGL function pointers
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		return -1;
	}
}

int main()
{
	initGLFW();
	//Initialise boids, walls, objects
	createLevel(nrBoids);
	
	// boid vertices
	glm::vec3 p0(-1.0f, -1.0f, 0.0f);
	glm::vec3 p1(0.0f, 1.5f, sqrt(3)/3);
	glm::vec3 p2(1.0f, -1.0f, 0.0f);
	glm::vec3 p3(0.0f, -1.0f, sqrt(3));

	// generate things
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
		
	// Build and compile shaders
	Shader shader("simple.vert", "simple.frag");

	// use (bind) the shader 1 so that we can attach matrices
	shader.use();

	// instantiate transformation matrices
	glm::mat4 projection, view, model;
	// projection will always be the same: define FOV, aspect ratio and view frustum (near & far plane)
	// set projection matrix as uniform (attach to bound shader)
	shader.setVec3("lightColor", lightColor);
	shader.setVec3("lightPos", lightPos);

	// instantiate array for boids
	glm::vec3 renderBoids[nrBoids*24]; // Each boid has three points and RGB color

	// Dear ImGui setup
	ImGui::CreateContext();
	ImGui::StyleColorsDark();
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init("#version 330"); // glsl version
	fps = ImGui::GetIO().Framerate;
	// Enable z-test 
	glEnable(GL_DEPTH_TEST);

	// render loop
	// -----------
	while (!glfwWindowShouldClose(window))
	{
		float currentFrame = glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;

		b4 = glfwGetTime();
		frame++;
		// Need to choose shader since we now have 2
		shader.use();
		// if got input, processed here
		processInput(window);

		// Setup frame for ImGui
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		// update fov
		projection = glm::perspective(glm::radians(fov), (float)screenWidth / screenHeight, 0.1f, 1000.0f);
		shader.setMatrix("projection", projection);
		// set view matrix
		view = glm::lookAt(cameraPos, cameraPos + cameraDir, cameraUp);
		// clear whatever was on screen last frame

		glClearColor(0.05f, 0.05f, 0.05f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Put all boids in the hash table so we can use it in the next loop
		for (Boid& b : boids){
			putInHashTable(b);
		}
		
		for (int i = 0; i < nrBoids; i++)
		{
			// Calculate new velocities for each boid, update pos given velocity
			Boid& b = boids[i];
			b.velocity += getSteering(b);
			b.velocity = normalize(b.velocity) * 0.001f;
			b.position += b.velocity;

			// create model matrix from agent position
			glm::mat4 model = glm::mat4(1.0f);
			model = glm::translate(model, b.position);
			glm::vec3 v = glm::vec3(b.velocity.z, 0, -b.velocity.x);
			float angle = acos(b.velocity.y / glm::length(b.velocity));
			model = glm::rotate(model, angle, v);

			shader.setMatrix("mv", view * model);
			// calculate transformed points of pyramid
			glm::vec3 v0 = glm::vec4(p0, 1.0f);
			glm::vec3 v1 = glm::vec4(p1, 1.0f);
			glm::vec3 v2 = glm::vec4(p2, 1.0f);
			glm::vec3 v3 = glm::vec4(p3, 1.0f);
			// calculate normals for each triangle
			glm::vec3 n0 = glm::cross(v2 - v0, v1 - v0);
			glm::vec3 n1 = glm::cross(v0 - v3, v1 - v3);
			glm::vec3 n2 = glm::cross(v3 - v2, v1 - v2);
			glm::vec3 n3 = glm::cross(v3 - v0, v2 - v0);

			// transform each vertex and add them to array
			renderBoids[i * 24 + 0] = v0;
			renderBoids[i * 24 + 1] = n0;
			renderBoids[i * 24 + 2] = v1;
			renderBoids[i * 24 + 3] = n0;
			renderBoids[i * 24 + 4] = v2;
			renderBoids[i * 24 + 5] = n0;

			renderBoids[i * 24 + 6] = v0;
			renderBoids[i * 24 + 7] = n1;
			renderBoids[i * 24 + 8] = v3;
			renderBoids[i * 24 + 9] = n1;
			renderBoids[i * 24 + 10] = v1;
			renderBoids[i * 24 + 11] = n1;

			renderBoids[i * 24 + 12] = v1;
			renderBoids[i * 24 + 13] = n2;
			renderBoids[i * 24 + 14] = v3;
			renderBoids[i * 24 + 15] = n2;
			renderBoids[i * 24 + 16] = v2;
			renderBoids[i * 24 + 17] = n2;

			renderBoids[i * 24 + 18] = v0;
			renderBoids[i * 24 + 19] = n3;
			renderBoids[i * 24 + 20] = v2;
			renderBoids[i * 24 + 21] = n3;
			renderBoids[i * 24 + 22] = v3;
			renderBoids[i * 24 + 23] = n3;
		};

		clearHashTable();
		// draw boids
		shader.use();
		glBindVertexArray(VAO);
		// bind buffer object and boid array
		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, nrBoids * sizeof(glm::vec3) * 24, &renderBoids[0], GL_STATIC_DRAW);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(0);

		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
		glEnableVertexAttribArray(1);

		// Draw 3 * nrBoids vertices
		shader.setVec3("color", glm::vec3(0.5f, 1.0f, 1.0f));
		glDrawArrays(GL_TRIANGLES, 0, nrBoids * 12);
	
		shader.setVec3("color", glm::vec3(0.0f));
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		glDrawArrays(GL_TRIANGLES, 0, nrBoids * 12);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

		// unbind buffer and vertex array
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);

		// ImGui create/render window
		createImGuiWindow();
		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		// glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
		glfwSwapBuffers(window);
		glfwPollEvents();
		timeElapsed += glfwGetTime() - b4;
		if (frame % 10 == 0) avgTimeElapsed = timeElapsed / frame;
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
void processInput(GLFWwindow* window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);

	float cameraSpeed = 250 * deltaTime;
	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		cameraPos += cameraSpeed * cameraDir;
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		cameraPos -= cameraSpeed * cameraDir;
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		cameraPos -= glm::normalize(glm::cross(cameraDir, cameraUp)) * cameraSpeed;
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		cameraPos += glm::normalize(glm::cross(cameraDir, cameraUp)) * cameraSpeed;
}

void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
	if (firstMouse)
	{
		lastX = xpos;
		lastY = ypos;
		firstMouse = false;
	}

	float xoffset = xpos - lastX;
	float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top
	lastX = xpos;
	lastY = ypos;

	float sensitivity = 0.1f; // change this value to your liking
	xoffset *= sensitivity;
	yoffset *= sensitivity;

	yaw += xoffset;
	pitch += yoffset;

	// make sure that when pitch is out of bounds, screen doesn't get flipped
	if (pitch > 89.0f)
		pitch = 89.0f;
	if (pitch < -89.0f)
		pitch = -89.0f;

	glm::vec3 front;
	front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
	front.y = sin(glm::radians(pitch));
	front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
	cameraDir = glm::normalize(front);
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	// make sure the viewport matches the new window dimensions; note that width and 
	// height will be significantly larger than specified on retina displays.
	glViewport(0, 0, width, height);
}