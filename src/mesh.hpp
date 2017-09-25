//
// Created by Jack Purvis
//

#ifndef POSITIONBASEDDYNAMICS_MESH_HPP
#define POSITIONBASEDDYNAMICS_MESH_HPP

#include <set>
#include <string>
#include <vector>
#include <Eigen>
#include <camera.hpp>
#include <GL/glew.h>

using namespace std;
using namespace Eigen;

struct Vertex {
    int p;
    int t;
    int n;
};

struct Edge {
    Edge(Vertex vertexA, Vertex vertexB) {
        v[0] = vertexA;
        v[1] = vertexB;
    }
    Vertex v[2];
};

struct Triangle {
    Vertex v[3];
};

enum ConstraintType {
    FIXED,
    DISTANCE
};

struct Constraint {
    int cardinatlity;
    vector<int> indices;
    ConstraintType type;
    Vector3f target;
    float distance = 0.0f;
    float stiffness = 1.0f;
};

class Mesh {

public:
    Mesh(string filename, Vector3f colour);
    void parseObjFile(string filename);
    void generateSurfaceNormals();

    void reset();
    void render(Camera* camera, Matrix4f transform);

    int numVertices;
    int numFaces;

    Vector3f position = Vector3f(0.0f, 0.0f, 0.0f);

    // Mesh fields
    vector<Vector3f> initialVertices;
    vector<Vector3f> vertices;
    vector<Vector2f> uvs;
    vector<Vector3f> normals;
    vector<Edge> edges;
    vector<Triangle> triangles;
    std::vector<Vector3f> surfaceNormals;

    // Simulation fields
    vector<Vector3f> velocities;
    vector<float> inverseMasses;
    vector<Vector3f> estimatePositions;
    vector<Constraint> constraints;

    // VBOs
    GLuint positionVBO;
    GLuint normalVBO;

    // Rendering
    GLuint shader;
    Vector3f colour;

};

#endif //POSITIONBASEDDYNAMICS_MESH_HPP
