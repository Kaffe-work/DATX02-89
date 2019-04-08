#include "glad/glad.h"
#include <GLFW/glfw3.h>
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>
#include "Shader.h"
#include <list>
#include "boid.h"
#include "obstacle.h"
#include "forceobject.h"
#include "levelfactory.h"
#include "spatial_hash.hpp"
#include <algorithm>
#include "tbb/parallel_for.h"
#include "tbb/task_scheduler_init.h"

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow *window);
double xpos, ypos; // cursor position

// setup
const unsigned int screenWidth = 1280, screenHeight = 720;

glm::vec3 cameraDir(1.0f, 1.0f, 200.0f);
glm::vec3 cameraPos(1.0f, 1.0f, -200.0f);
double yaw = 1.6f, pitch = 0.0f;

// How many boids on screen
const int nrBoids = 5000;

// Which level
const int level = 1;

// Level attributes
std::vector<Boid> boids;
std::vector<Obstacle> walls;
std::vector<ForceObject> objects;

// Boid attributes
const float MAX_SPEED = 0.3f;
const float MAX_ACCELERATION = 0.05f;
const float SOFTNESS = 10.0f;
bool repellLine = false;

// Time, used to print performance
double lastTime = glfwGetTime();
int nrFrames = 0;

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

// If e.g rangePercent is 5 then this will return a number between 0.95 and 1.05
float getRandomFloatAroundOne(int rangePercent) {
	return 1.0f + ((rand() % 1001 - 500) % (rangePercent * 10)) / 1000.0f;
}

glm::vec3 getSteering(Boid & b) { // Flocking rules are implemented here

	glm::vec3 alignment = glm::vec3(0.0);
	glm::vec3 separation = glm::vec3(0.0);
	glm::vec3 cohesion = glm::vec3(0.0);
	glm::vec3 repellation = glm::vec3(0.0);
	glm::vec3 avoidance = glm::vec3(0.0);
	glm::vec3 chemistry = glm::vec3(0.0);
	std::vector<Boid*> nb = getNeighbours(b);

	for (Boid* n : nb) {
		Boid neighbour = *n;
		alignment += neighbour.velocity;
		cohesion += neighbour.position;
		//separation += normalize(b.position - neighbour.position) * SOFTNESS / (pow(distance(b.position, neighbour.position),2) + 0.0001); // + 0.0001 is for avoiding divide by zero
		separation += normalize(b.position - neighbour.position) / distance(b.position, neighbour.position);
	}

	if (std::size(nb) > 0) {
		alignment = normalize(alignment * (1.0f / std::size(nb)) - b.velocity);
		cohesion = normalize(cohesion * (1.0f / std::size(nb)) - b.position - b.velocity);
		separation = normalize(separation * (1.0f / std::size(nb)) - b.velocity);
	}

	//Avoid walls
	for (Obstacle o : walls) {
		glm::vec3 v = b.position - o.point;
		float distance = SOFTNESS / glm::dot(v, o.normal);
		avoidance += normalize(o.normal)*distance - b.velocity;
	}

	//Avoid/steer towards a force point
	for (ForceObject f : objects) {
		if (f.attractive) {
			chemistry -= normalize(b.position - f.position) / distance(b.position, f.position);
		}
		else {
			chemistry += normalize(b.position - f.position) / distance(b.position, f.position);
		}
	}
	if (std::size(objects) > 0) {
		chemistry = normalize(chemistry * (1.0f / std::size(objects)) - b.velocity);
	}

	
	if (repellLine) {
		glm::vec3 point = cameraPos + dot(b.position - cameraPos, cameraDir) / dot(cameraDir, cameraDir) * (cameraDir);
		//repellation = normalize(b.position - point) / (pow(distance(b.position, point) / SOFTNESS + 2.0f, 2)) - b.velocity;
		repellation = normalize(b.position - point) * pow(SOFTNESS,2) / (distance(b.position, point)) - b.velocity;
	}

	

	glm::vec3 steering = alignment + cohesion + 2.0f*separation + 10.0f*avoidance + 10.0f*chemistry + repellation;
	
	// Limit acceleration
	float magnitude = glm::clamp(glm::length(steering), 0.0f, MAX_ACCELERATION); 
	return magnitude*glm::normalize(steering);

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

	// build and compile shader program
	Shader shader("vert.shader", "frag.shader");

	//Initialise boids, walls, objects
	boids = getLevelBoids(level, nrBoids);
	walls = getLevelWalls(level);
	objects = getLevelObjects(level);

	// one vector for each vertex
	glm::vec3 p1(-1.0f, -1.0f, 0.0f);
	glm::vec3 p2(0.0f, 1.0f, 0.0f);
	glm::vec3 p3(1.0f, -1.0f, 0.0f);

	// generate vertex array object
	unsigned int VAO, VBO;
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);

	// use the shader created earlier so we can attach matrices
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

		tbb::parallel_for( 
			tbb::blocked_range<size_t>(0, nrBoids),
			[&](const tbb::blocked_range<size_t>& r) {
			for (size_t i = r.begin(); i < r.end(); ++i)
			{
				// Calculate new velocities for each boid, update pos given velocity
				boids[i].velocity += getSteering(boids.at(i));
				boids[i].velocity = normalize(boids[i].velocity)*MAX_SPEED;
				boids[i].position += boids[i].velocity; 

				// create model matrix from agent position
				glm::mat4 model = glm::mat4(1.0f);
				model = glm::translate(model, boids[i].position);
				glm::vec3 v = glm::vec3(boids[i].velocity.z, 0, -boids[i].velocity.x);
				float angle = acos(boids[i].velocity.y / glm::length(boids[i].velocity));
				model = glm::rotate(model, angle, v);

				// transform each vertex and add them to array
				renderBoids[i*3] = view * model * glm::vec4(p1, 1.0f);
				renderBoids[i*3 + 1] = view * model * glm::vec4(p2, 1.0f);
				renderBoids[i*3 + 2] = view * model * glm::vec4(p3, 1.0f);
			}
		} );

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
	if (state == GLFW_PRESS) {
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