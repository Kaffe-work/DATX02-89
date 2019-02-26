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
int nr_boids = 10;
std::vector<Boid> boids;

/* SPATIAL HASHING STUFF */

// Grid related stuff
#define CELL_SIZE 100.0f // Boids currently have a width of 2.0 (?)
const int HASH_TABLE_SIZE = 10;

// node structure for single linked list
template<typename T>
struct node{
   T value;
   node<T>* next;
   node() : value(NULL), next(NULL) {}
   node(T t) : next(NULL){
	   value = t;
   } 
};

// HashTable with all the boids
std::vector<node<Boid*> > cellBuckets(HASH_TABLE_SIZE, node<Boid*>()); // Initialize to NULL

template<typename T>
void insertNode(std::vector<node<T> > &table, int index, T value){
	if(index >= table.size()) {
		std::cout << "Hashtable: Index out of bounds";
		return;
	}
	node<T>* n = &table[index];
	if(n->value == NULL){ // Bucket is empty
		table[index] = node<T>(value);
		return;
	}
	while(n->next != NULL){ // Some nodes already in bucket
		n = n->next;
	}
	n->next = new node<T>(value);
}

template<typename T>
void clearHashTable(std::vector<node<T> > &table){
	for(node<T>& n : table){
		node<T>* next;
		node<T>* current = n.next;
		while (current != NULL){
			next = current->next;
			current->value = NULL;
			delete current;
			current = next;
		} 
		n.value = NULL;
		n.next = NULL; // not strictly needed, but can help avoid future bugs
	}
}

template <typename T>
void printBuckets(std::vector<node<T> > &buckets){
	for(int i = 0; i < buckets.size(); i++){
		node<T>* n = &buckets[i];
		if(n->value == NULL) continue; // skip empty buckets
		std::cout << std::endl << "Bucket nr " << i << ": ";
		while(n != NULL){
			std::cout << "*";
			n = n->next;
		}
	}
}


inline glm::vec3 getCell(glm::vec3 pos){
	return pos * (1.0f/CELL_SIZE);  
}

int getCellHash(glm::vec3 cell){
	const glm::vec3 primes = glm::vec3(402653189, 805306457, 1610612741); // one prime per coordinate
	cell = glm::floor(cell) * primes;
	int hash = ((int)cell.x ^ (int)cell.y ^ (int)cell.z) % HASH_TABLE_SIZE; 
	return std::abs(hash); 
}

typedef node<Boid*> BoidNode; // More handy way of refering to the nodes?

BoidNode* getNeighbours(Boid& b){
	int index = getCellHash(getCell(b.position));
	return &cellBuckets[index];
}

void putInHashTable(Boid& b){
	glm::vec3 cell = getCell(b.position); // which cell is the boid in
	int hash = getCellHash(cell); // hash the cell position
	insertNode(cellBuckets, hash, &b); // wrap the boid in a node and put in hash table
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

			glDrawArrays(GL_TRIANGLE_STRIP, 0, sizeof(boidModel) / (sizeof(float) * 3));
		}

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