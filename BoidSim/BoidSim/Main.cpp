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
#include <algorithm>
#include "kernel.h"
#include <random>

// Enable timing
//#define TIMING
// #define FRAMES_MEASURED 50

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void processInput(GLFWwindow *window);
std::string random_string(size_t length);

GLFWwindow* window;


double xpos, ypos; // cursor position

// setup
const unsigned int screenWidth = 1920, screenHeight = 1080;
int printscreenNr = 0;

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

// How many boids on screen
const int nrBoids = NR_BOIDS;
const int nrPredators = 500;
Boid* boids;

// lighting
glm::vec3 lightPos(350.0f, 700.0f, 350.0f);
glm::vec3 lightPos2(50.0f, 700.0f, 350.0f);

// Time, used to print performance
double lastTime = glfwGetTime();
int nrFrames = 0;


// for printscreens
bool printscreen(std::string filename, int w, int h)
{
	//This prevents the images getting padded 
   // when the width multiplied by 3 is not a multiple of 4
	glPixelStorei(GL_PACK_ALIGNMENT, 1);

	int nSize = w * h * 3;
	// First let's create our buffer, 3 channels per Pixel
	char* dataBuffer = (char*)malloc(nSize * sizeof(char));

	if (!dataBuffer) return false;

	// Let's fetch them from the backbuffer	
	// We request the pixels in GL_BGR format, thanks to Berzeger for the tip
	glReadPixels((GLint)0, (GLint)0,
		(GLint)w, (GLint)h,
		GL_BGR, GL_UNSIGNED_BYTE, dataBuffer);

	//Now the file creation
	FILE* filePtr = fopen(filename.c_str(), "wb");
	if (!filePtr) return false;


	unsigned char TGAheader[12] = { 0,0,2,0,0,0,0,0,0,0,0,0 };
	unsigned char header[6] = { w % 256,w / 256,
					h % 256,h / 256,
					24,0 };
	// We write the headers
	fwrite(TGAheader, sizeof(unsigned char), 12, filePtr);
	fwrite(header, sizeof(unsigned char), 6, filePtr);
	// And finally our image data
	fwrite(dataBuffer, sizeof(GLubyte), nSize, filePtr);
	fclose(filePtr);

	free(dataBuffer);

	return true;
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
    bool maybe = percentage == 0 ? false : rand() % (100 / percentage) == 0;
    return glm::vec3(maybe ? rand() % 121 - 60, rand() % 121 - 60, rand() % 21 - 10 : 0, 0, 0);
}

int frames = 0;

int initGLFW()
{
	// glfw: initialize and configure
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // Needed for OS X, and possibly Linux
	glfwWindowHint(GLFW_SAMPLES, 8); // smoothen edges

														 // glfw: window creation
	window = glfwCreateWindow(screenWidth, screenHeight, "BoidSim", NULL /*glfwGetPrimaryMonitor()*/, NULL);
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

	double updateTime = 0.;
	double renderTime = 0.;

    boids = *(initBoidsOnGPU(boids));
    // glfw: initialize and configure
	initGLFW();

    // build and compile shader program
    Shader shader("vert.shader", "frag.shader");
	long rando = 1;

	// build and compile shader program
	Shader lampShader("lamp.vert", "lamp.frag");

	float vertices[] = {
		-0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
		0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
		0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
		0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
		-0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
		-0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,

		-0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
		0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
		0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
		0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
		-0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
		-0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,

		-0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,
		-0.5f,  0.5f, -0.5f, -1.0f,  0.0f,  0.0f,
		-0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,
		-0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,
		-0.5f, -0.5f,  0.5f, -1.0f,  0.0f,  0.0f,
		-0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,

		0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,
		0.5f,  0.5f, -0.5f,  1.0f,  0.0f,  0.0f,
		0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,
		0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,
		0.5f, -0.5f,  0.5f,  1.0f,  0.0f,  0.0f,
		0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,

		-0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,
		0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,
		0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,
		0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,
		-0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,
		-0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,

		-0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,
		0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,
		0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
		0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
		-0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
		-0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f
	};

	std::random_device rd;
	std::uniform_real_distribution<double> dist(CELL_SIZE + 1.f, MAX_COORD - CELL_SIZE);
	std::uniform_real_distribution<double> dist2(-MAX_SPEED, MAX_SPEED);
	//Create prey and predators
	for (int i = 0; i < nrBoids; i++) {
		srand(i);
		boids[i] = Boid();
		
		boids[i].position.x = dist(rd);
		boids[i].position.y = dist(rd);
		boids[i].position.z = dist(rd);
		boids[i].velocity.x = dist2(rd);
		boids[i].velocity.y = dist2(rd);
		boids[i].velocity.z = dist2(rd);
		
	}
	for (int i = 0; i < nrPredators; i++) {
		boids[i].status |= PREDATOR_FLAG;
	}

    unsigned int VAO;
    GLuint positionsVBO;
    glGenVertexArrays(1, &VAO); // is this needed?
    struct cudaGraphicsResource* positionsVBO_CUDA;

    // Explicitly set device 0
    cudaSetDeviceWrapper(0);

    // Create buffer object and register it with CUDA
    glGenBuffers(1, &positionsVBO);
    glBindBuffer(GL_ARRAY_BUFFER, positionsVBO);
    unsigned int size = nrBoids * sizeof(glm::vec3) * 54;
    glBufferData(GL_ARRAY_BUFFER, size, 0, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    cudaGraphicsGLRegisterBufferWrapper(&positionsVBO_CUDA, positionsVBO);

    // use the shader created earlier so we can attach matrices
    shader.use();

    // instantiate transformation matrices
    glm::mat4 projection, view, model;
    // projection will always be the same: define FOV, aspect ratio and view frustum (near & far plane)
    projection = glm::perspective(glm::radians(45.0f), (float)screenWidth / screenHeight, 0.1f, 1000.0f);
    // set projection matrix as uniform (attach to bound shader)
    shader.setMatrix("projection", projection);
	// For lightning
	shader.setVec3("lightPos", lightPos);
	shader.setVec3("lightPos2", lightPos2);

	// also draw the lamp object
	lampShader.use();
	lampShader.setMatrix("projection", projection);
	unsigned int lightVAO, VBO2;
	glGenVertexArrays(1, &lightVAO);
	glGenBuffers(1, &VBO2);
	glBindVertexArray(lightVAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO2);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	// Enable z-test 
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_MULTISAMPLE); // smoother edges

    // render loop
    // -----------
    while (!glfwWindowShouldClose(window))
    {
		float currentFrame = glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;
        // print performance to console
        // printPerformance();
        // if got input, processed here
        processInput(window);

        // update camera direction, rotation
		projection = glm::perspective(glm::radians(fov), (float)screenWidth / screenHeight, 0.1f, 1000.0f);

		shader.use();
		shader.setMatrix("projection", projection);
		// set view matrix
		view = glm::lookAt(cameraPos, cameraPos + cameraDir, cameraUp);
		shader.setMatrix("view", view);
        // clear whatever was on screen last frame
		glm::vec3 bgColor(5.f/255.f, 30.f/255.f, 62.f/255.f);
        glClearColor(bgColor.x, bgColor.y, bgColor.z, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);


		double lastTime = glfwGetTime();
        step(); // one step in the simulation on the gpu
		updateTime += glfwGetTime()- lastTime;



		lastTime = glfwGetTime();



        // Map buffer object so CUDA can access OpenGl buffer
        glm::vec3* renderBoids;
        size_t num_bytes;
        mapBufferObjectCuda(&positionsVBO_CUDA, &num_bytes, &renderBoids);

        //Execute kernel HERE 
        prepareBoidRender(boids, renderBoids, projection, view);
        printCUDAError();

        // Unmap buffer object so OpenGl can access the buffer again
        cudaGraphicsUnmapResourcesWrapper(&positionsVBO_CUDA);

        // Render from buffer object
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Bind buffer object and boid array
        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, positionsVBO);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)(3 * sizeof(float)));
		glEnableVertexAttribArray(1);

		glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)(6 * sizeof(float)));
		glEnableVertexAttribArray(2);

        // Draw 3 * nrBoids vertices
		shader.setVec3("color", glm::vec3(1.0f, 1.0f, 1.0f));
		shader.setVec3("bgColor", bgColor);
		shader.setVec3("cameraPos", cameraPos);
        glDrawArrays(GL_TRIANGLES, 0, nrBoids * 24);


		// Draw light source
		lampShader.use();
		lampShader.setMatrix("view", view);
		glm::mat4 lightModel = glm::scale(glm::translate(glm::mat4(1.0f), lightPos), glm::vec3(20.f));
		lampShader.setMatrix("model", lightModel);

		glBindVertexArray(lightVAO);
		glDrawArrays(GL_TRIANGLES, 0, 36);

		glm::mat4 lightModel2 = glm::scale(glm::translate(glm::mat4(1.0f), lightPos2), glm::vec3(20.f));
		lampShader.setMatrix("model", lightModel2);
		glBindVertexArray(lightVAO);
		glDrawArrays(GL_TRIANGLES, 0, 36);

		// enable for wireframe
		/*
		shader.setVec3("color", glm::vec3(0.0f));
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		glDrawArrays(GL_TRIANGLES, 0, nrBoids * 24);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		*/
        // unbind buffer and vertex array
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);

        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        glfwSwapBuffers(window);
        glfwPollEvents();

#ifdef TIMING
		renderTime += glfwGetTime() - lastTime;
		if (frames++ > FRAMES_MEASURED) break;
#endif
    }


    cudaGraphicsUnregisterResourceWrapper(positionsVBO_CUDA);
    glDeleteBuffers(1, &positionsVBO);

    // glfw: terminate, clearing all previously allocated GLFW resources.
    // glfw: terminate, clearing all previously allocated GLFW resour
    glfwTerminate();
    deinitBoidsOnGPU();
#ifdef TIMING
	printf("Update time: %f\n", updateTime / (double)FRAMES_MEASURED);
	printf("Render time: %f\n", renderTime / (double)FRAMES_MEASURED);
	while (1) {}
#endif
	
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
	if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS)
		printscreen("screens/" + std::to_string(printscreenNr++) + ".tga", screenWidth, screenHeight);
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