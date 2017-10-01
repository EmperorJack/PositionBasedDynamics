//
// Created by Jack Purvis
//

#include <iostream>
#include <fstream>
#include <sstream>
#include <shaderLoader.hpp>
#include <mesh.hpp>

Mesh::Mesh(string filename, Vector3f colour) : colour(colour) {
    parseObjFile(filename);

    this->numVertices = (int) vertices.size();
    this->numFaces = (int) triangles.size();

    initialVertices = vertices;

    // Setup VBO
    glGenBuffers(1, &positionVBO);
    glGenBuffers(1, &normalVBO);

    // Setup shader
    shader = loadShaders("SimpleVertexShader", "SimpleFragmentShader");

    // Setup simulation
    reset();
    inverseMasses.resize((size_t) numVertices, 1.0f);
}

Mesh::~Mesh() {
    for (Constraint* constraint : constraints) {
        delete constraint;
    }
}

void Mesh::generateSurfaceNormals() {
    surfaceNormals.clear();
    for (int i = 0; i < triangles.size(); i++) {
        Vector3f v1 = vertices[triangles[i].v[1].p] - vertices[triangles[i].v[0].p];
        Vector3f v2 = vertices[triangles[i].v[2].p] - vertices[triangles[i].v[0].p];
        Vector3f surfaceNormal = v1.cross(v2);
        surfaceNormal.normalize();
        surfaceNormals.push_back(surfaceNormal);
    }
}

void Mesh::reset() {
    vertices = initialVertices;

    estimatePositions.clear();
    velocities.clear();

    Vector3f initialVelocity(1.0f, 0.0f, -1.0f);
    velocities.resize((size_t) numVertices, initialVelocity);
}

void Mesh::render(Camera* camera, Matrix4f transform) {

    // Setup transform
    Affine3f t(Translation3f(position[0], position[1], position[2]));
    Matrix4f modelMatrix = transform * t.matrix();

    // Compute vertex normals
    vector<Vector3f> tempNormals;
    tempNormals.resize((size_t) numVertices, Vector3f::Zero());
    generateSurfaceNormals();
    for (unsigned int i = 0; i < triangles.size(); i++) {
        Triangle tri = triangles[i];

        for (int j = 0; j < 3; j++) {
            tempNormals[tri.v[j].p] += surfaceNormals[i];
        }
    }

    // Build vertex positions and normals
    vector<Vector3f> outVertices;
    vector<Vector3f> outNormals;
    for (unsigned int i = 0; i < triangles.size(); i++) {
        Triangle tri = triangles[i];

        for (int j = 0; j < 3; j++) {
            Vector3f position = vertices[tri.v[j].p];
            Vector3f normal = tempNormals[tri.v[j].p];

            outVertices.push_back(position);
            normal.normalize();
            outNormals.push_back(normal);
        }
    }

    glBindBuffer(GL_ARRAY_BUFFER, positionVBO);
    glBufferData(GL_ARRAY_BUFFER, outVertices.size() * sizeof(Vector3f), outVertices[0].data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, normalVBO);
    glBufferData(GL_ARRAY_BUFFER, outNormals.size() * sizeof(Vector3f), outNormals[0].data(), GL_STATIC_DRAW);

    glUseProgram(shader);

    glUniform3fv(glGetUniformLocation(shader, "materialColour"), 1, colour.data());
    Vector4f lightPosition = Vector4f(8, 10, 0, 0);
    lightPosition = modelMatrix * lightPosition;
    glUniform3fv(glGetUniformLocation(shader, "lightPosition"), 1, lightPosition.data());

    // Bind matrices
    glUniformMatrix4fv(glGetUniformLocation(3, "projection"), 1, GL_FALSE, camera->projectionMatrix.data());
    glUniformMatrix4fv(glGetUniformLocation(3, "view"), 1, GL_FALSE, camera->viewMatrix.data());
    glUniformMatrix4fv(glGetUniformLocation(3, "model"), 1, GL_FALSE, modelMatrix.data());

    // Bind vertex positions
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, positionVBO);
    glVertexAttribPointer(
            0,         // shader layout attribute
            3,         // size
            GL_FLOAT,  // type
            GL_FALSE,  // normalized?
            0,         // stride
            (void*)0   // array buffer offset
    );

    // Bind vertex normals
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, normalVBO);
    glVertexAttribPointer(
            1,         // shader layout attribute
            3,         // size
            GL_FLOAT,  // type
            GL_FALSE,  // normalized?
            0,         // stride
            (void*)0   // array buffer offset
    );

    glDrawArrays(GL_TRIANGLES, 0, outVertices.size());
    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
}

void Mesh::parseObjFile(string filename) {

    // Attempt to open an input stream to the file
    ifstream objFile(filename);
    if (!objFile.is_open()) {
        cout << "Error reading " << filename << endl;
        return;
    }

    // While the file has lines remaining
    while (objFile.good()) {

        // Pull out line from file
        string line;
        getline(objFile, line);
        istringstream objLine(line);

        // Pull out mode from line
        string mode;
        objLine >> mode;

        // Reading like this means whitespace at the start of the line is fine
        // attempting to read from an empty string/line will set the failbit
        if (!objLine.fail()) {

            if (mode == "v") { // Vertex
                Vector3f v;
                objLine >> v[0] >> v[1] >> v[2];
                vertices.push_back(v);
            } else if (mode == "vn") { // Vertex normal
                Vector3f vn;
                objLine >> vn[0] >> vn[1] >> vn[2];
                normals.push_back(vn);
            } else if (mode == "vt") { // UV
                Vector2f vt;
                objLine >> vt[0] >> vt[1];
                uvs.push_back(vt);
            } else if (mode == "f") { // Face
                vector<Vertex> verts;
                while (objLine.good()) {
                    Vertex v;

                    // OBJ face is formatted as v/vt/vn
                    objLine >> v.p; // Scan in position index
                    objLine.ignore(1); // Ignore the '/' character
                    objLine >> v.t; // Scan in uv (texture coord) index
                    objLine.ignore(1); // Ignore the '/' character
                    objLine >> v.n; // Scan in normal index

                    // Correct the indices
                    v.p -= 1;
                    v.n -= 1;
                    v.t -= 1;

                    verts.push_back(v);
                }

                // If we have 3 vertices, construct a triangle
                if (verts.size() >= 3) {
                    Triangle triangle;
                    triangle.v[0] = verts[0];
                    triangle.v[1] = verts[1];
                    triangle.v[2] = verts[2];
                    triangles.push_back(triangle);

                    // Construct edges
                    Edge e1 = Edge(verts[0], verts[1]);
                    Edge e2 = Edge(verts[0], verts[2]);
                    Edge e3 = Edge(verts[1], verts[2]);
                    edges.insert(e1);
                    edges.insert(e2);
                    edges.insert(e3);

                    // Add to adjacent triangles
                    adjacentTriangles[e1].push_back(triangle);
                    adjacentTriangles[e2].push_back(triangle);
                    adjacentTriangles[e3].push_back(triangle);
                }
            }
        }
    }

    generateSurfaceNormals();
}
