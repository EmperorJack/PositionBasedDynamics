//
// Created by Jack Purvis
//

#ifndef POSITIONBASEDDYNAMICS_MESH_HPP
#define POSITIONBASEDDYNAMICS_MESH_HPP

#include <map>
#include <set>
#include <string>
#include <vector>
#include <Eigen>
#include <GL/glew.h>
#include <bounding_box.hpp>
#include <camera.hpp>

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

struct EdgeCompare {
    std::less<std::pair<int, int>> lessComparator;
    bool operator() (const Edge &l, const Edge &r) const {
        return lessComparator(minmax(l.v[0].p, l.v[1].p), minmax(r.v[0].p, r.v[1].p));
    }
};

struct Triangle {
    Vertex v[3];
};

class Mesh {

public:
    Mesh(string filename, Vector3f colour, float inverseMass = 1.0f);
    ~Mesh();
    void reset();
    void applyImpulse(Vector3f force);
    void translate(Vector3f translate);
    bool intersect(Vector3f rayOrigin, Vector3f rayDirection, float &t, Vector3f &normal, int vertexIndex, int &triangleIndex);
    void updateBoundingBox();
    void render(Camera* camera, Matrix4f transform);

    int numVertices;
    int numFaces;

    Vector3f position = Vector3f(0.0f, 0.0f, 0.0f);

    // Mesh fields
    vector<Vector3f> initialVertices;
    vector<Vector3f> vertices;
    vector<Vector2f> uvs;
    vector<Vector3f> normals;
    set<Edge, EdgeCompare> edges;
    vector<Triangle> triangles;
    map<Edge, vector<Triangle>, EdgeCompare> adjacentTriangles;
    std::vector<Vector3f> surfaceNormals;

    // Simulation fields
    vector<Vector3f> velocities;
    float inverseMass;
    int estimatePositionsOffset;
    bool isRigidBody = false;
    bool gravityAffected = false;
    bool windAffected = false;

private:
    bool rayTriangleIntersect(Vector3f rayOrigin, Vector3f rayDirection, float &t, int triangleIndex, int vertexIndex, float &u, float &v);
    void parseObjFile(string filename);
    void generateSurfaceNormals();

    // VBOs
    GLuint positionVBO;
    GLuint normalVBO;

    // Rendering
    GLuint shader;
    Vector3f colour;

    // Bounding box
    BoundingBox* boundingBox;

};

#endif //POSITIONBASEDDYNAMICS_MESH_HPP
