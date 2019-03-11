#include "spatial_hash.hpp"
#include <tuple>
#include <vector>
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
std::vector<Boid*> nextBoid(nrBoids, NULL);

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

void clearHashTable(){
    cellBuckets.clear();
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