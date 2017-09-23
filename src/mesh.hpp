//
// Created by Jack Purvis
//

#ifndef POSITIONBASEDDYNAMICS_MESH_HPP
#define POSITIONBASEDDYNAMICS_MESH_HPP

#include <string>
#include <vector>
#include <Eigen>

using namespace std;
using namespace Eigen;

struct Vertex {
    int p;
    int t;
    int n;
};

struct Triangle {
    Vertex v[3];
};

class Mesh {

public:
    Mesh(string filename);
    void parseObjFile(string filename);

    int numVertices;
    int numFaces;

    // Mesh fields
    vector<Vector3f> vertices;
    vector<Triangle> triangles;
    vector<Vector2f> uvs;
    vector<Vector3f> normals;
    std::vector<Vector3f> surfaceNormals;

    // Simulation fields
    vector<Vector3f> velocities;
    vector<float> masses;

};

#endif //POSITIONBASEDDYNAMICS_MESH_HPP
