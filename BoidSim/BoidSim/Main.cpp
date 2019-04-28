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

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow *window);
double xpos, ypos; // cursor position

GLFWwindow* window;

bool cameraReset = true; 
const int nrBoids = 100000;
const unsigned int screenWidth = 1280, screenHeight = 720;
glm::vec3 cameraDir(1.0f, 1.0f, 200.0f);
glm::vec3 cameraPos(1.0f, 1.0f, -200.0f);
double yaw = 1.6f, pitch = 1.0f;
unsigned int VAO, VBO;
int frame = 0;
float fps;
float avgFps = 0;

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

	//Flee predators and check death
	if (size(predators) > 0) {
		for (Boid n : predators) {
			flee += 1.0f/n.position;
			if (distance(n.position, b.position) < DEATH_DISTANCE) {
				b.isAlive = false;
				scoreNegative++;
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

void createImGuiWindow()
{
	static float f = 0.0f;
	static int counter = 0;

	ImGui::Begin("Performance:");

	ImGui::Text("Application average %.3f ms/frame (%.1f FPS), avgFps: %.1f", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate, avgFps);

	ImGui::End();
}

void resetCamera() {
	cameraDir = glm::vec3(1.0f, 1.0f, 200.0f);
	cameraPos = glm::vec3(1.0f, 1.0f, -200.0f);
	yaw = 1.6f;
	pitch = 0.0f;
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
	window = glfwCreateWindow(screenWidth, screenHeight, "BoidSim", NULL, NULL);
	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

	// GLFW catches the cursor
	//glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

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
	projection = glm::perspective(glm::radians(45.0f), (float)screenWidth / screenHeight, 0.1f, 1000.0f);
	// set projection matrix as uniform (attach to bound shader)
	shader.setMatrix("projection", projection);

	// instantiate array for boids
	glm::vec3 renderBoids[nrBoids*24]; // Each boid has three points and RGB color

	// Dear ImGui setup
	ImGui::CreateContext();
	ImGui::StyleColorsDark();
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init("#version 330"); // glsl version
	fps = ImGui::GetIO().Framerate;


	// render loop
	// -----------
	while (!glfwWindowShouldClose(window))
	{
		fps += ImGui::GetIO().Framerate;
		frame++;
		if (frame % 100 == 0) avgFps = fps / frame; frame = 0;
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
			if (boids[i].isPredator) {
				boids[i].velocity += getSteeringPredator(boids.at(i));
				boids[i].velocity = normalize(boids[i].velocity) * MAX_SPEED_PREDATOR;
			}
			else {
				boids[i].velocity += getSteeringPrey(boids.at(i));
				boids[i].velocity = normalize(boids[i].velocity) * MAX_SPEED;
			}

			boids[i].position += boids[i].velocity;


			// create model matrix from agent position
			glm::mat4 model = glm::mat4(1.0f);
			model = glm::translate(model, boids[i].position);
			if (boids[i].isPredator) model = glm::scale(model, glm::vec3(2.0f));
			glm::vec3 v = glm::vec3(boids[i].velocity.z, 0, -boids[i].velocity.x);
			float angle = acos(boids[i].velocity.y / glm::length(boids[i].velocity));
			model = glm::rotate(model, angle, v);

			glm::vec3 color = glm::vec3(1.0f) * (float)!boids[i].isPredator;
			// transform each vertex and add them to array
			renderBoids[i * 24 + 0] = view * model * glm::vec4(p0, 1.0f);
			renderBoids[i * 24 + 1] = color;
			renderBoids[i * 24 + 2] = view * model * glm::vec4(p1, 1.0f);
			renderBoids[i * 24 + 3] = color;
			renderBoids[i * 24 + 4] = view * model * glm::vec4(p2, 1.0f);
			renderBoids[i * 24 + 5] = color;

			renderBoids[i * 24 + 6] = view * model * glm::vec4(p0, 1.0f);
			renderBoids[i * 24 + 7] = color;
			renderBoids[i * 24 + 8] = view * model * glm::vec4(p3, 1.0f);
			renderBoids[i * 24 + 9] = color;
			renderBoids[i * 24 + 10] = view * model * glm::vec4(p1, 1.0f);
			renderBoids[i * 24 + 11] = color;

			renderBoids[i * 24 + 12] = view * model * glm::vec4(p1, 1.0f);
			renderBoids[i * 24 + 13] = color;
			renderBoids[i * 24 + 14] = view * model * glm::vec4(p3, 1.0f);
			renderBoids[i * 24 + 15] = color;
			renderBoids[i * 24 + 16] = view * model * glm::vec4(p2, 1.0f);
			renderBoids[i * 24 + 17] = color;

			renderBoids[i * 24 + 18] = view * model * glm::vec4(p0, 1.0f);
			renderBoids[i * 24 + 19] = color;
			renderBoids[i * 24 + 20] = view * model * glm::vec4(p2, 1.0f);
			renderBoids[i * 24 + 21] = color;
			renderBoids[i * 24 + 22] = view * model * glm::vec4(p3, 1.0f);
			renderBoids[i * 24 + 23] = color;
		}

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