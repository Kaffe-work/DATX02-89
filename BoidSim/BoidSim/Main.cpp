// #include "OpenGL/gl.h"
#include "glad/glad.h"
#include <GLFW/glfw3.h>
#include "Shader.h"
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>
#include <map>



void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow *window);

// setup
const unsigned int screenWidth = 1280;
const unsigned int screenHeight = 720;

// A boid has a position on screen and a velocity. Constructor creates random vec3 of both
struct Boid {
	glm::vec3 position, velocity;

	Boid()
		: position(rand() % 21 - 10, rand() % 21 - 10, 0.0f), velocity(rand() % 21 - 10, rand() % 21 - 10, 0.0f) { }
};

// How many boids on screen
int nr_boids = 50;
std::vector<Boid> boids;

/* SPATIAL HASHING STUFF */

// Grid related stuff
#define CELL_SIZE 10.0f // Boids currently have a width of 2.0 (?)
const int HASH_TABLE_SIZE = 100;

struct BoidBucket{
	Boid *head, *tail;
	BoidBucket() : head(NULL), tail(NULL) {}
    BoidBucket(Boid* b){
	   head = b;
       tail = b;
    }
};

// HashTable with all the boids
std::vector<BoidBucket> cellBuckets(HASH_TABLE_SIZE, BoidBucket()); 
// Table containing one (if any) cell neighbour for each boid 
std::vector<Boid*> nextBoid(nr_boids, NULL);

// Helper function to convert a boids absolute address to it's offset in the vector containing all boids
inline int absToOffset(Boid* b){
    return ((long)b - (long)&boids[0])/sizeof(Boid); 
} 

// Put in the linked list for bucket with index
void putInBucket(Boid& boid, int index){
    if(index >= cellBuckets.size()) {
		std::cout << "Hashtable: Index out of bounds";
		return;
	}
    if(cellBuckets[index].head == NULL){ // Bucket is empty
        cellBuckets[index] = BoidBucket(&boid);
    } 
    Boid* oldTail = cellBuckets[index].tail;
    cellBuckets[index].tail = &boid;
    int i = absToOffset(oldTail);// check offset from boids vector base (="index" of boid in vector)
    nextBoid[i] = &boid; // the old tail boid now points to the new tail
}

void clearHashTable(){
    for(BoidBucket &b : cellBuckets){
        b.head = NULL;
        b.tail = NULL; // tail = NULL not strictly needed, only to prevent future bugs
    }
}

void printBuckets(){
    for(int i = 0; i < cellBuckets.size(); i++){
        if(cellBuckets[i].head == NULL) {
            continue; // Empty bucket, don't print anything nothing
        }
		std::cout << std::endl << "Bucket nr " << i << ": " << absToOffset(cellBuckets[i].head);
        Boid* tail = cellBuckets[i].tail;
        Boid* current = cellBuckets[i].head;
        do{
            int j = absToOffset(current);
            current = nextBoid[j];
            std::cout << "->" << absToOffset(current);
        }
        while(current != tail);
    }
	std::cout << std::endl;
}

inline glm::vec3 getCell(glm::vec3 pos){
	return pos * (1.0f/CELL_SIZE);  
}

// Computes the hash for a cell
int getCellHash(glm::vec3 cell){
	const glm::vec3 primes = glm::vec3(402653189, 805306457, 1610612741); // one prime per coordinate
	cell = glm::floor(cell) * primes;
	int hash = ((int)cell.x ^ (int)cell.y ^ (int)cell.z) % HASH_TABLE_SIZE; 
	return std::abs(hash); 
}

// Puts boid b in the correct place in the hash table
void putInHashTable(Boid& b){
	glm::vec3 cell = getCell(b.position); // which cell is the boid currently in
	int hashIndex = getCellHash(cell); // hash the cell position
	putInBucket(b, hashIndex); // put in hash table at index hashIndex
}


int main()
{   
	// glfw: initialize and configure
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // Needed for OS X

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
	
	// create the boids (allocate space and randomize position/velocity by calling constructor)
	for (int i = 0; i < nr_boids; ++i)
		boids.push_back(Boid());
	
	// the model of a boid (just a triangle, three vertices), same for all boids ofc
	float boidModel[] = {
		-1.0f, -1.0f, 0.0f,
		 0.0f,  1.0f, 0.0f,
		 1.0f, -1.0f, 0.0f
	};

	unsigned int VBO, VAO;
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(boidModel), boidModel, GL_STATIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	
	// render loop
	// -----------

    int frame = 0;
    
	while (!glfwWindowShouldClose(window))
	{
		// if got input, processed here
		processInput(window);
		
		// use the shader instantiated earlier
		shader.use();
		
		// create some matrices for our coordinate system
		glm::mat4 view = glm::mat4(1.0f);
		// view: zoom out camera
		view = glm::translate(view, glm::vec3(0.0f, 0.0f, -120.0f));
		glm::mat4 projection;
		// projection: define FOV, aspect ratio and view frustum (near & far plane)
		projection = glm::perspective(glm::radians(45.0f), (float)screenWidth / screenHeight, 0.1f, 150.0f);

		// attach the specified matrices to our shader as uniforms (send them to vertex shader)
		shader.setMatrix("view", view);
		shader.setMatrix("projection", projection);
        shader.setVec3("ourColor", glm::vec3(0.0f));
		// clear whatever was on screen last frame
		glClearColor(0.90f, 0.95f, 0.96f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);
		glBindVertexArray(VAO);

		// move each boid to current pos, update pos given velocity
		for (Boid& b : boids)
		{
			b.position += 0.005f * b.velocity;
			glm::mat4 model = glm::mat4(1.0f);
			// rotate boid to face correct location (doesn't work)
			//float angle = acos(dot(glm::vec3(0.0f, 1.0f, 0.0f), normalize(b.velocity)));
			//if(angle > 1) model = glm::rotate(model, angle, glm::vec3(0.0f, 0.0f, 1.0f));
			// move the model to boid location
			model = glm::translate(model, b.position);
			// each boid gets its unique uniform model (applied in vertex shader)
			shader.setMatrix("model", model);
            // std::cout << "putting in hashtable: " << &b;
            putInHashTable(b);
			glDrawArrays(GL_TRIANGLE_STRIP, 0, sizeof(boidModel) / (sizeof(float) * 3));
		}
        printBuckets();
		clearHashTable();

		// render the triangle

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