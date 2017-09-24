//
// Created by Jack Purvis
//

#include <iostream>
#include <fstream>
#include <sstream>
#include <mesh.hpp>

Mesh::Mesh(string filename) {
    parseObjFile(filename);

    this->numVertices = (int) vertices.size();
    this->numFaces = (int) triangles.size();

    // Setup VBO
    glGenBuffers(1, &meshVBO);
    glBindBuffer(GL_ARRAY_BUFFER, meshVBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vector3f), vertices[0].data(), GL_STATIC_DRAW);
}

void Mesh::render() {
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

//    cout << numVertices << endl;

    glDrawArrays(GL_TRIANGLES, 0, 24 * 3);
    glDisableVertexAttribArray(0);
}

void Mesh::parseObjFile(string filename) {
    vector<Vector3f> tempVertices;
    vector<Vector2f> tempUvs;
    vector<Vector3f> tempNormals;

    // Load dummy vertices because OBJ indexing starts at 1 not 0
    Vector3f v = Vector3f(0, 0, 0);
    Vector2f uv = Vector2f(0, 0);
    Vector3f n = Vector3f(0, 0, 1);
    tempVertices.push_back(v);
    tempUvs.push_back(uv);
    tempNormals.push_back(n);

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

            if (mode == "v") {
                // Parse vertex
                Vector3f v;
                objLine >> v[0] >> v[1] >> v[2];
                tempVertices.push_back(v);
            } else if (mode == "vn") {
                // Parse vertex normal
                Vector3f vn;
                objLine >> vn[0] >> vn[1] >> vn[2];
                tempNormals.push_back(vn);
            } else if (mode == "vt") {
                // Parse UV
                Vector2f vt;
                objLine >> vt[0] >> vt[1];
                tempUvs.push_back(vt);
            } else if (mode == "f") {
                // Parse face
                vector<Vertex> verts;
                while (objLine.good()) {
                    Vertex v;

                    objLine >> v.p; // Scan in position index

                    // Check if the obj file contains normal data
                    if (tempNormals.size() > 1) {
                        // obj face is formatted as v/vt/vn or v//vn
                        objLine.ignore(1); // Ignore the '/' character

                        if (tempUvs.size() > 1) { // Check if the obj file contains uv data
                            objLine >> v.t; // Scan in uv (texture coord) index
                        }
                        objLine.ignore(1); // Ignore the '/' character

                        objLine >> v.n; // Scan in normal index
                    } // Else the obj face is formatted just as v

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
        Vector3f v1 = tempVertices[triangles[i].v[1].p] - tempVertices[triangles[i].v[0].p];
        Vector3f v2 = tempVertices[triangles[i].v[2].p] - tempVertices[triangles[i].v[0].p];
        Vector3f surfaceNormal = v1.cross(v2);
        surfaceNormal.normalize();
        surfaceNormals.push_back(surfaceNormal);
    }

    // Perform indexing
    for (unsigned int i = 0; i < triangles.size(); i++) {
        Triangle tri = triangles[i];

        for (int j = 0; j < 3; j++) {
            vertices.push_back(tempVertices[tri.v[j].p]);
        }
    }
}