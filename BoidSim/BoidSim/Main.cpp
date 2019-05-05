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
void processInput(GLFWwindow *window);
double xpos, ypos; // cursor position

GLFWwindow* window;

bool cameraReset = true; 
const int nrBoids = 1000;
const unsigned int screenWidth = 1280, screenHeight = 720;
glm::vec3 cameraDir(1.0f, 1.0f, 200.0f);
glm::vec3 cameraPos(1.0f, 1.0f, -200.0f);
double yaw = 1.6f, pitch = 1.0f;
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
		
		
		tbb::parallel_for(
			tbb::blocked_range<size_t>(0, nrBoids),
			[&](const tbb::blocked_range<size_t>& r) {
			for (size_t i = r.begin(); i < r.end(); ++i)
			{
				// Calculate new velocities for each boid, update pos given velocity
				Boid& b = boids[i];
				b.velocity += getSteering(b);
				b.velocity = normalize(b.velocity) * MAX_SPEED;
				b.position += b.velocity;

				// create model matrix from agent position
				glm::mat4 model = glm::mat4(1.0f);
				model = glm::translate(model, b.position);
				glm::vec3 v = glm::vec3(b.velocity.z, 0, -b.velocity.x);
				float angle = acos(b.velocity.y / glm::length(b.velocity));
				model = glm::rotate(model, angle, v);

				// transform each vertex and add them to array
				renderBoids[i * 12 + 0] = view * model * glm::vec4(p0, 1.0f);
				renderBoids[i * 12 + 1] = view * model * glm::vec4(p1, 1.0f);
				renderBoids[i * 12 + 2] = view * model * glm::vec4(p2, 1.0f);

				renderBoids[i * 12 + 3] = view * model * glm::vec4(p0, 1.0f);
				renderBoids[i * 12 + 4] = view * model * glm::vec4(p3, 1.0f);
				renderBoids[i * 12 + 5] = view * model * glm::vec4(p1, 1.0f);

				renderBoids[i * 12 + 6] = view * model * glm::vec4(p1, 1.0f);
				renderBoids[i * 12 + 7] = view * model * glm::vec4(p3, 1.0f);
				renderBoids[i * 12 + 8] = view * model * glm::vec4(p2, 1.0f);

				renderBoids[i * 12 + 9] = view * model * glm::vec4(p0, 1.0f);
				renderBoids[i * 12 + 10] = view * model * glm::vec4(p2, 1.0f);
				renderBoids[i * 12 + 11] = view * model * glm::vec4(p3, 1.0f);
			}
		});

		clearHashTable();
		// draw boids
		shader.use();
		glBindVertexArray(VAO);
		// bind buffer object and boid array
		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, nrBoids * sizeof(glm::vec3) * 12, &renderBoids[0], GL_STATIC_DRAW);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(0);

		// Draw 3 * nrBoids vertices
		shader.setVec3("color", glm::vec3(1.0f));
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