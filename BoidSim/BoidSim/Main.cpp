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
#include "levelfactory.h"
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

// setup
const unsigned int screenWidth = 1280, screenHeight = 720;
GLFWwindow* window;

// camera settings
glm::vec3 cameraDir(1.0f, 1.0f, 200.0f);
glm::vec3 cameraPos(1.0f, 1.0f, -200.0f);
double yaw = 1.6f, pitch = 0.0f;

// How many boids on screen
const int nrBoids = 10;

// Which level
const int level = 1;

// Level attributes
std::vector<Boid> boids;

// Boid attributes
const float MAX_SPEED = 0.3f;
const float MAX_ACCELERATION = 0.05f;
const float SOFTNESS = 10.0f;
bool repellLine = false;

// Compute shader stuff
GLuint flock_buffer[2], flock_render_vao[2];
GLuint geometry_buffer, flock_update_program, flock_render_program;
GLuint frame_index = 0;
enum
{
	WORKGROUP_SIZE = 256,
	NUM_WORKGROUPS = 4000,
	FLOCK_SIZE = (NUM_WORKGROUPS * WORKGROUP_SIZE)
};
static const glm::vec3 geometry[] =
{
	// Positions
	glm::vec3(-5.0f, 1.0f, 0.0f),
	glm::vec3(-1.0f, 1.5f, 0.0f),
	glm::vec3(-1.0f, 1.5f, 7.0f),
	glm::vec3(0.0f, 0.0f, 0.0f),
	glm::vec3(0.0f, 0.0f, 10.0f),
	glm::vec3(1.0f, 1.5f, 0.0f),
	glm::vec3(1.0f, 1.5f, 7.0f),
	glm::vec3(5.0f, 1.0f, 0.0f),

	// Normals
	glm::vec3(0.0f),
	glm::vec3(0.0f),
	glm::vec3(0.107f, -0.859f, 0.00f),
	glm::vec3(0.832f, 0.554f, 0.00f),
	glm::vec3(-0.59f, -0.395f, 0.00f),
	glm::vec3(-0.832f, 0.554f, 0.00f),
	glm::vec3(0.295f, -0.196f, 0.00f),
	glm::vec3(0.124f, 0.992f, 0.00f),
};
struct flock_member
{
	glm::vec3 position;
	unsigned int : 32;
	glm::vec3 velocity;
	unsigned int : 32;
};

float b4, timeElapsed, avgElapsed;
GLuint64 time;



// If e.g. percentage = 1 => vec3(0,0,0) will be returned with 99% probability
glm::vec3 getRandomVectorWithChance(int percentage) {
	bool maybe = percentage == 0 ? false : rand() % (100/percentage) == 0;
	return glm::vec3(maybe ? rand() % 121 - 60, rand() % 121 - 60, rand() % 21 - 10 : 0, 0, 0);
}

// If e.g rangePercent is 5 then this will return a number between 0.95 and 1.05
float getRandomFloatAroundOne(int rangePercent) {
	return 1.0f + ((rand() % 1001 - 500) % (rangePercent * 10)) / 1000.0f;
}

/*glm::vec3 getSteering(Boid & b) { // Flocking rules are implemented here

	glm::vec3 alignment = glm::vec3(0.0);
	glm::vec3 separation = glm::vec3(0.0);
	glm::vec3 cohesion = glm::vec3(0.0);
	glm::vec3 lineforce = glm::vec3(0.0);
	glm::vec3 planeforce = glm::vec3(0.0);
	glm::vec3 pointforce = glm::vec3(0.0);
	std::vector<Boid*> nb = getNeighbours(b);

	//Flocking rules
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

	//Avoid planes
	for (ObstaclePlane o : walls) {
		glm::vec3 v = b.position - o.point;
		float distance = SOFTNESS / glm::dot(v, o.normal);
		planeforce += normalize(o.normal)*distance - b.velocity;
	}

	//Avoid/steer towards an obstaclepoint
	for (ObstaclePoint f : objects) {
		if (f.attractive) {
			pointforce -= normalize(b.position - f.position) / distance(b.position, f.position);
		}
		else {
			pointforce += normalize(b.position - f.position) / distance(b.position, f.position);
		}
	}
	if (std::size(objects) > 0) {
		pointforce = normalize(pointforce * (1.0f / std::size(objects)) - b.velocity);
	}

	//Avoid player controlled line
	if (repellLine) {
		glm::vec3 point = cameraPos + dot(b.position - cameraPos, cameraDir) / dot(cameraDir, cameraDir) * (cameraDir);
		lineforce = normalize(b.position - point) * pow(SOFTNESS,2) / (distance(b.position, point)) - b.velocity;
	}

	glm::vec3 steering = alignment + cohesion + 2.0f*separation + 10.0f*planeforce + 10.0f*pointforce + lineforce;
	
	// Limit acceleration
	float magnitude = glm::clamp(glm::length(steering), 0.0f, MAX_ACCELERATION); 
	return magnitude*glm::normalize(steering);

}
*/

void setupCSBuffers() {

	glGenBuffers(2, flock_buffer);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, flock_buffer[0]);
	glBufferData(GL_SHADER_STORAGE_BUFFER, FLOCK_SIZE * sizeof(flock_member), NULL, GL_DYNAMIC_COPY);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, flock_buffer[1]);
	glBufferData(GL_SHADER_STORAGE_BUFFER, FLOCK_SIZE * sizeof(flock_member), NULL, GL_DYNAMIC_COPY);

	int i;

	glGenBuffers(1, &geometry_buffer);
	glBindBuffer(GL_ARRAY_BUFFER, geometry_buffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(geometry), geometry, GL_STATIC_DRAW);

	glGenVertexArrays(2, flock_render_vao);

	for (i = 0; i < 2; i++)
	{
		glBindVertexArray(flock_render_vao[i]);
		glBindBuffer(GL_ARRAY_BUFFER, geometry_buffer);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)(8 * sizeof(glm::vec3)));

		glBindBuffer(GL_ARRAY_BUFFER, flock_buffer[i]);
		glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(flock_member), NULL);
		glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(flock_member), (void*)sizeof(glm::vec4));
		glVertexAttribDivisor(2, 1);
		glVertexAttribDivisor(3, 1);

		glEnableVertexAttribArray(0);
		glEnableVertexAttribArray(1);
		glEnableVertexAttribArray(2);
		glEnableVertexAttribArray(3);
	}
}

int initGLFW()
{
	// glfw: initialize and configure
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
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

void createImGuiWindow()
{
	static float f = 0.0f;
	static int counter = 0;

	ImGui::Begin("Performance");                          // Create a window called "Hello, world!" and append into it.


	ImGui::Text("Time elapsed %f for updating", avgElapsed);
	ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
	ImGui::End();
}

int main()
{
	initGLFW();

	// Compute shader
	Shader flock_update_program("flocking.comp");
	Shader flock_render_program("render.vert", "render.frag");

	//Initialise boids, walls, objects
	//boids = getLevelBoids(level, nrBoids);
	setupCSBuffers();

	glBindBuffer(GL_ARRAY_BUFFER, flock_buffer[0]);
	flock_member* ptr = reinterpret_cast<flock_member*>(glMapBufferRange(GL_ARRAY_BUFFER, 0, FLOCK_SIZE * sizeof(flock_member), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT));

	for (int i = 0; i < FLOCK_SIZE; i++)
	{
		ptr[i].position = glm::vec3(rand() % 500 - 250, rand() % 500 - 250, rand() % 500 - 250);
		ptr[i].velocity = glm::vec3(rand() % 500 - 250, rand() % 500 - 250, rand() % 500 - 250);
	}


	glm::mat4 projection, view, model;
	// projection will always be the same: define FOV, aspect ratio and view frustum (near & far plane)
	projection = glm::perspective(glm::radians(45.0f), (float)screenWidth / screenHeight, 0.1f, 1000.0f);
	glUnmapBuffer(GL_ARRAY_BUFFER);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	// render loop
	int t = 0;
	GLuint64 timer, timer2;
	GLint64 timer1;
	unsigned int query;
	int done = 0;
	glGenQueries(1, &query);

	// Dear ImGui setup
	ImGui::CreateContext();
	ImGui::StyleColorsDark();
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init("#version 330"); // glsl version

	while (!glfwWindowShouldClose(window))
	{
		glQueryCounter(query, GL_TIMESTAMP);

		// get the current time
		glGetInteger64v(GL_TIMESTAMP, &timer1);

		t++;
		// Need to choose shader since we now have 2
		flock_update_program.use();
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

		glm::vec3 goal(sinf(0.34f), cosf(0.29f), sinf(0.12f)* cosf(0.5f));
		goal = goal * glm::vec3(15.0f, 15.0f, 180.0f);
		flock_update_program.setVec3("goal", goal);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, flock_buffer[frame_index]);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, flock_buffer[frame_index ^ 1]);

		glDispatchCompute(NUM_WORKGROUPS, 1, 1);

		glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		timeElapsed = glfwGetTime() - b4;

		flock_render_program.use();
		glm::mat4 mvp = projection * view;

		flock_render_program.setMatrix("mvp", mvp);
		glBindVertexArray(flock_render_vao[frame_index]);
		glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 8, FLOCK_SIZE);
		frame_index ^= 1;

		glEndQuery(GL_TIME_ELAPSED);
		glGetQueryObjectui64v(query, GL_QUERY_RESULT, &timer2);
		timeElapsed += (timer2 - timer1) / 1000000.0;

		// ImGui create/render window
		createImGuiWindow();
		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		// glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
		glfwSwapBuffers(window);
		glfwPollEvents();
		if (t % 2 == 0) avgElapsed = timeElapsed / t;
	}

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
		repellLine = true;
	}else {
		repellLine = false;
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