//
// Created by Jack Purvis
//

#include <iostream>
#include <fstream>
#include <sstream>
#include <shaderLoader.hpp>
#include <mesh.hpp>

Mesh::Mesh(string filename, Vector4f colour) : colour(colour) {
    parseObjFile(filename);

    this->numVertices = (int) vertices.size();
    this->numFaces = (int) triangles.size();

    // Setup VBO
    glGenBuffers(1, &meshVBO);

    // Setup shader
    shader = loadShaders("SimpleVertexShader", "SimpleFragmentShader");
}

void Mesh::render(Camera* camera, Matrix4f transform) {

    // Setup transform
    Affine3f t(Translation3f(position[0], position[1], position[2]));
    Matrix4f modelMatrix = transform * t.matrix();

    // Build renderable vertex list
    vector<Vector3f> outVertices;

    for (unsigned int i = 0; i < triangles.size(); i++) {
        Triangle tri = triangles[i];

        for (int j = 0; j < 3; j++) {
            outVertices.push_back(vertices[tri.v[j].p]);
        }
    }
    glBindBuffer(GL_ARRAY_BUFFER, meshVBO);
    glBufferData(GL_ARRAY_BUFFER, outVertices.size() * sizeof(Vector3f), outVertices[0].data(), GL_STATIC_DRAW);

    glUseProgram(shader);

    glUniform4fv(glGetUniformLocation(shader, "fillColor"), 1, colour.data());

    // Bind matrices
    glUniformMatrix4fv(glGetUniformLocation(3, "projection"), 1, GL_FALSE, camera->projectionMatrix.data());
    glUniformMatrix4fv(glGetUniformLocation(3, "view"), 1, GL_FALSE, camera->viewMatrix.data());
    glUniformMatrix4fv(glGetUniformLocation(3, "model"), 1, GL_FALSE, modelMatrix.data());

    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, meshVBO);
    glVertexAttribPointer(
            0,         // shader layout attribute
            3,         // size
            GL_FLOAT,  // type
            GL_FALSE,  // normalized?
            0,         // stride
            (void*)0   // array buffer offset
    );

    glDrawArrays(GL_TRIANGLES, 0, outVertices.size());
    glDisableVertexAttribArray(0);
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
                    Triangle tri;
                    tri.v[0] = verts[0];
                    tri.v[1] = verts[1];
                    tri.v[2] = verts[2];
                    triangles.push_back(tri);
                }
            }
        }
    }

    // Generate surface normals
    for (int i = 0; i < triangles.size(); i++) {
        Vector3f v1 = vertices[triangles[i].v[1].p] - vertices[triangles[i].v[0].p];
        Vector3f v2 = vertices[triangles[i].v[2].p] - vertices[triangles[i].v[0].p];
        Vector3f surfaceNormal = v1.cross(v2);
        surfaceNormal.normalize();
        surfaceNormals.push_back(surfaceNormal);
    }
}