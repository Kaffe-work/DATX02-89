// #include "OpenGL/gl.h"
#include "glad/glad.h"
#include <GLFW/glfw3.h>

#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>
#include "flat_hash_map.hpp"
#include <Shader.h>
#include <list>


void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow *window);
double xpos, ypos; // cursor position

// setup
const unsigned int screenWidth = 1280, screenHeight = 720;

glm::vec3 cameraDir(1.0f, 1.0f, 200.0f);
glm::vec3 cameraPos(1.0f, 1.0f, -200.0f);
double yaw = 1.6f, pitch = 0.0f;

// A boid has a position on screen and a velocity. Constructor creates random vec3 of both
struct Boid {
	glm::vec3 position, velocity;

	Boid()
		: position(rand() % 81 - 40, rand() % 81 - 40, 0), velocity(rand() % 61 - 30, rand() % 61 - 30, 0) { }
};

// How many boids on screen
int nr_boids = 100;
std::vector<Boid> boids;

// Boid attributes
const float MAX_SPEED = 20.0f;
const float MIN_SPEED = 7.0f;


int frame = 0;

/* SPATIAL HASHING STUFF */
#include <tuple>
using std::tuple;

#define USE_SPATIAL_HASH // comment out this for the naive n^2 version
// Grid related stuff
const float CELL_SIZE = 10.0f; // this should be the same value as the boids scope
const int HASH_TABLE_SIZE = 997;

struct BoidBucket{
	Boid *head, *tail;
	BoidBucket() : head(NULL), tail(NULL) {}
    BoidBucket(Boid* b){
	   head = tail = b;
    }
	BoidBucket(Boid* a, Boid* b){
		head = a;
		tail = b;
	}
};

// HashTable with all the boids
ska::flat_hash_map<unsigned long, BoidBucket> cellBuckets = ska::flat_hash_map<unsigned long, BoidBucket>();
// Table containing one (if any) cell neighbour for each boid 
std::vector<Boid*> nextBoid(nr_boids, NULL);

// Helper function to convert a boids absolute address to it's offset in the vector containing all boids
inline int absToOffset(Boid* b){
    return ((long)b - (long)&boids[0])/sizeof(Boid); 
} 

// This part is for hashing tuples of ints. Standard hash maps can only hash enum types
template <class T>
inline void hash_combine(std::size_t &seed, T &v)
{
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

// Hashes tuples of ints
long getCellHash(tuple<int, int, int> cell){
	size_t seed = 0;
    hash_combine(seed, std::get<0>(cell));
    hash_combine(seed, std::get<1>(cell));
	hash_combine(seed, std::get<2>(cell));
    return seed;
}

inline tuple<int, int, int> getCell(glm::vec3 pos){
	glm::vec3 cell = glm::floor(pos * (1.0f/CELL_SIZE));
	return tuple<int,int,int>(cell.x, cell.y, cell.z);  
}

// Puts boid b in the correct place in the hash table
void putInHashTable(Boid& b){
	tuple<int, int, int> cell = getCell(b.position); // which cell is the boid currently in
	size_t cellHash = getCellHash(cell);
	auto iter = cellBuckets.find(cellHash);
	if(iter != cellBuckets.end()){
		Boid* head = iter->second.head;
		Boid* oldTail = iter->second.tail;
   		iter->second = BoidBucket(head, &b);
    	int i = absToOffset(oldTail);// check offset from boids vector base (="index" of boid in vector)
    	nextBoid[i] = &b; // the old tail boid now points to the new tail
	} else {
		cellBuckets.insert(std::pair<size_t, BoidBucket>(cellHash, BoidBucket(&b)));
	}
	
}

// A little helper function that checks if b is within a's scope
inline bool validNeighbour(Boid& a, Boid& b){
	if(a.position != b.position && distance(a.position, b.position) < 10.0f){
		return true;
	}
	return false;
}

std::vector<Boid*> getNeighbours(Boid& b){
	// check all 3*3 neighbouring cells for boids
	tuple<int, int,int> cell = getCell(b.position); 
	// Collect all neighbours in a vector. Future optimization: iterate over neighbours directly instead of collecting in vector
	std::vector<Boid*> neighbours; 
	int boidIndex = absToOffset(&b);
	for(int i= -1; i <= 1; i++){
		for(int j= -1; j <= 1; j++){
			for(int k= -1; k <= 1; k++){
				tuple<int, int,int> neighbourCell = {std::get<0>(cell)+i, std::get<1>(cell)+j, std::get<2>(cell)+k}; 
				auto iter = cellBuckets.find(getCellHash(neighbourCell));
				if(iter != cellBuckets.end()){
					Boid* current = iter->second.head;
					Boid* tail = iter->second.tail;
					if(validNeighbour(b, *current)){
						neighbours.push_back(current);
					}
					while(current != tail){
						int l = absToOffset(current);
						current = nextBoid[l];
						if(validNeighbour(b, *current)){
							neighbours.push_back(current); 
						}
					}
				} 
			}
		}
	}
	return neighbours;
}


void updateBoids(Boid & b) { // Flocking rules are implemented here
	
	/*Alignment = Velocity Matching*/
	//Sum the velocities of the neighbours and this boid and average them.

	/*Separation = Collision Avoidance*/
	//Sum the vectors from all neighbours to this boid. 

	/*Cohesion - Flock Centering*/
	//Sum the positions of the neighbours and average them, then subtract this boids position
	
	glm::vec3 alignment = b.velocity;
	glm::vec3 separation = glm::vec3(0.0);
	glm::vec3 cohesion = glm::vec3(0.0);

	#ifdef USE_SPATIAL_HASH
	std::vector<Boid*> nb = getNeighbours(b);
	#else 
	std::vector<Boid> nb;
	for (Boid a : boids) {
		comparisons++;
		if ((a.position != b.position) && distance(a.position, b.position) < 10.0f)
		{
			nb.push_back(a);
		}
	}
	#endif

	// EXPERIMENTAL! Limit the boids to a dome 
	float distFromOrigo = length(b.position);
	const float domeRadius = 80;
	const float margin = 20;
	bool approachingLimit = false;
	float speed;
	glm::vec3 domeAvoidance = glm::vec3(0);
	if(distFromOrigo + margin > domeRadius && length(b.position + b.velocity) > distFromOrigo){
		speed = length(b.velocity);
		approachingLimit = true;
		domeAvoidance = std::max(0.0f, (domeRadius - distFromOrigo)/margin)*b.velocity - (margin/std::max(0.001f, domeRadius - distFromOrigo))*b.position;
	}

	if (std::size(nb) == 0) { // If there are no neighbours, dont change speed
		return;
	}
	#ifndef USE_SPATIAL_HASH
	for (Boid neighbour : nb) {
	#else 
	for(Boid* c : nb) {
		Boid neighbour = *c;
	#endif
		alignment += neighbour.velocity;
		separation += (b.position - neighbour.position) * 1.0f/(float)(pow(distance(b.position, neighbour.position),2) + 0.0001); // + 0.0001 is for avoiding divide by zero
		cohesion += neighbour.position;
	}
	alignment = alignment * (1.0f / (std::size(nb) + 1));
	cohesion = cohesion * (1.0f / std::size(nb)) - b.position;
	separation = separation * (1.0f / std::size(nb));

	glm::vec3 newVel;
	/*Update Velocity*/
	if(approachingLimit){ // This is for the dome
		newVel = domeAvoidance + separation; 
	} else {
		newVel = alignment + 50.0f*separation + cohesion;
	}
	speed = glm::clamp(length(newVel), MIN_SPEED, MAX_SPEED); // limit speed
	b.velocity = speed*glm::normalize(newVel);
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
	for (int i = 0; i < nrBoids; ++i)
		boids.push_back(Boid());

	// the model of a boid (just a triangle, three vertices), same for all boids ofc
	float boidModel[] = {
		-0.67f, -1.0f, 0.0f,
		 0.0f,  1.0f, 0.0f,
		 0.67f, -1.0f, 0.0f
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
		// update camera direction, rotation
		cameraDir = glm::vec3(cos(pitch)*cos(yaw), sin(-pitch), cos(pitch)*sin(yaw));
		normalize(cameraDir);
		// calculate view-matrix based on cameraDir and cameraPos
		view = glm::lookAt(cameraPos, cameraPos + cameraDir, glm::vec3(0.0f, 1.0f, 0.0f));
		glm::mat4 projection;
		// projection: define FOV, aspect ratio and view frustum (near & far plane)
		projection = glm::perspective(glm::radians(45.0f), (float)screenWidth / screenHeight, 0.1f, 1000.0f);

		// attach the specified matrices to our shader as uniforms (send them to vertex shader)
		shader.setMatrix("view", view);
		shader.setMatrix("projection", projection);
		// clear whatever was on screen last frame
		glClearColor(0.90f, 0.95f, 0.96f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);
		glBindVertexArray(VAO);

		// move each boid to current pos, update pos given velocity
		#ifdef USE_SPATIAL_HASH
		for (Boid& b : boids){
			putInHashTable(b);
		}
		#endif
		for (Boid& b : boids)
		{
			updateBoids(b);
			tuple<int,int,int> cell = getCell(b.position);
			//shader.setVec3("ourColor", glm::vec3(1.0f/((int)cell.x>>4), 1.0f/((int)cell.y>>5), 1.0f/((int)cell.z>>6)));
			
			b.position += 0.01f * b.velocity;
			glm::mat4 model = glm::mat4(1.0f);
			
			// This seems to sovle the infamous rotating problem
			model = glm::translate(model, b.position); // move the boid to the correct position
			glm::vec3 v = glm::vec3(b.velocity.z, 0, -b.velocity.x);
			float angle = acos(b.velocity.y / glm::length(b.velocity));
			model = glm::rotate(model, angle, v);

			// each boid gets its unique uniform model (applied in vertex shader)
			shader.setMatrix("model", model);
			shader.setVec3("ourColor", glm::vec3(1.0f,1.0f,1.0f));

			glDrawArrays(GL_TRIANGLE_STRIP, 0, sizeof(boidModel) / (sizeof(float) * 3));
		}		
		cellBuckets.clear();

		// render the triangle

		// glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
		glfwSwapBuffers(window);
		glfwPollEvents();
		frame++;
		// if(frame == 500) break;
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
