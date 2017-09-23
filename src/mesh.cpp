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
}

void Mesh::parseObjFile(string filename) {

    // Load dummy vertices because OBJ indexing starts at 1 not 0
    Vector3f v = Vector3f(0, 0, 0);
    Vector2f uv = Vector2f(0, 0);
    Vector3f n = Vector3f(0, 0, 1);
    vertices.push_back(v);
    uvs.push_back(uv);
    normals.push_back(n);

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
                vertices.push_back(v);
            } else if (mode == "vn") {
                // Parse vertex normal
                Vector3f vn;
                objLine >> vn[0] >> vn[1] >> vn[2];
                normals.push_back(vn);
            } else if (mode == "vt") {
                // Parse UV
                Vector2f vt;
                objLine >> vt[0] >> vt[1];
                uvs.push_back(vt);
            } else if (mode == "f") {
                // Parse face
                vector<Vertex> verts;
                while (objLine.good()) {
                    Vertex v;

                    objLine >> v.p; // Scan in position index

                    // Check if the obj file contains normal data
                    if (normals.size() > 1) {
                        // obj face is formatted as v/vt/vn or v//vn
                        objLine.ignore(1); // Ignore the '/' character

                        if (uvs.size() > 1) { // Check if the obj file contains uv data
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
        Vector3f v1 = vertices[triangles[i].v[1].p] - vertices[triangles[i].v[0].p];
        Vector3f v2 = vertices[triangles[i].v[2].p] - vertices[triangles[i].v[0].p];
        Vector3f surfaceNormal = v1.cross(v2);
        surfaceNormal.normalize();
        surfaceNormals.push_back(surfaceNormal);
    }
}