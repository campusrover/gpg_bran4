#include <iostream>
#include <array>

class NodeHandle {
public:
    std::array<int, 10000> data;

    NodeHandle() {
        // Initialize the array to zero
        data.fill(0);
    }
};

class Arm {
public:
    NodeHandle* nodeHandle;

    Arm() : nodeHandle(nullptr) {}

    void setNodeHandle(NodeHandle& nh) {
        nodeHandle = &nh;
    }

};

// Global variable of type Arm
Arm globalArm;

int main() {
    // Allocate NodeHandle on the heap
    NodeHandle* nodeHandle = new NodeHandle();
    // Set the NodeHandle reference in globalArm
    globalArm.setNodeHandle(*nodeHandle);
    return 0;
}