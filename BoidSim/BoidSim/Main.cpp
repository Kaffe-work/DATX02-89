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
void processInput(GLFWwindow *window);
double xpos, ypos; // cursor position



// setup
const unsigned int screenWidth = 1280, screenHeight = 720;

glm::vec3 cameraDir(1.0f, 1.0f, 200.0f);
glm::vec3 cameraPos(1.0f, 1.0f, -200.0f);
double yaw = 1.6f, pitch = 0.0f;

// How many boids on screen
const int nrBoids = NR_BOIDS;
const int nrPredators = 100;
Boid* boids;


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
    bool maybe = percentage == 0 ? false : rand() % (100 / percentage) == 0;
    return glm::vec3(maybe ? rand() % 121 - 60, rand() % 121 - 60, rand() % 21 - 10 : 0, 0, 0);
}

int frames = 0;

int main()
{

	double updateTime = 0.;
	double renderTime = 0.;

    boids = *(initBoidsOnGPU(boids));
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
	long rando = 1;


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


    // one vector for each vertex
    glm::vec3 p1(-1.0f, -1.0f, 0.0f);
    glm::vec3 p2(0.0f, 1.0f, 0.0f);
    glm::vec3 p3(1.0f, -1.0f, 0.0f);

    unsigned int VAO;
    GLuint positionsVBO;
    glGenVertexArrays(1, &VAO); // is this needed?
    struct cudaGraphicsResource* positionsVBO_CUDA;

    // Explicitly set device 0
    cudaSetDeviceWrapper(0);

    // Create buffer object and register it with CUDA
    glGenBuffers(1, &positionsVBO);
    glBindBuffer(GL_ARRAY_BUFFER, positionsVBO);
    unsigned int size = nrBoids * 3 * sizeof(glm::vec3);
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


    // render loop
    // -----------
    while (!glfwWindowShouldClose(window))
    {
        // print performance to console
        // printPerformance();
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

#ifdef TIMING
		renderTime += glfwGetTime() - lastTime;
		if (frames++ > FRAMES_MEASURED) break;
#endif
    }


    cudaGraphicsUnregisterResourceWrapper(positionsVBO_CUDA);
    glDeleteBuffers(1, &positionsVBO);

    // glfw: terminate, clearing all previously allocated GLFW resources.
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