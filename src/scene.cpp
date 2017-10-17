//
// Created by Jack Purvis
//

#include <iostream>
#include <main.hpp>
#include <scene.hpp>

Scene::Scene() {

    // Setup camera
    camera = new Camera();

    // Setup scene configurations
    setupConfigurationA();
    setupConfigurationB();
    setupConfigurationC();
    configuration = configurationA;
}

Scene::~Scene() {
    delete camera;
    delete configurationA;
    delete configurationB;
    delete configurationC;
}

void Scene::reset() {
    for (Mesh* mesh : configuration->simulatedObjects) {
        mesh->reset();
    }
}

void Scene::setConfiguration(int index) {
    switch (index) {
        case 0:
            configuration = configurationA;
            break;
        case 1:
            configuration = configurationB;
            break;
        case 2:
            configuration = configurationC;
            break;
        default:
            break;
    }
}

void Scene::render(bool wireframe) {
    camera->setPerspective(45.0f, (float) SCREEN_WIDTH / (float) SCREEN_HEIGHT, 0.1f, 100.0f);
    camera->lookAt(Vector3f(0, 0, 20), Vector3f(0, 0, 0), Vector3f(0, 1, 0));

    AngleAxisf pitchAngle(pitch, Vector3f::UnitX());
    AngleAxisf yawAngle(yaw, Vector3f::UnitY());
    AngleAxisf rollAngle(roll, Vector3f::UnitZ());
    Quaternion<float> q = rollAngle * yawAngle * pitchAngle;
    Matrix4f r = Matrix4f::Identity();
    r.block(0, 0, 3, 3) = q.toRotationMatrix();
    Matrix4f modelMatrix = Matrix4f::Identity() * r;

    if (wireframe) glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    else glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    for (Mesh* mesh : configuration->staticObjects) {
        mesh->render(camera, modelMatrix);
    }

    for (Mesh* mesh : configuration->simulatedObjects) {
        mesh->render(camera, modelMatrix);
    }
}

void Scene::setupConfigurationA() {
    configurationA = new Configuration();

    Vector3f meshColour = { 0.15f, 0.45f, 0.8f };
    Mesh* testCube = new Mesh("../resources/models/cube.obj", meshColour);
    testCube->gravityAffected = true;
    testCube->isRigidBody = true;

    Vector3f planeColour = { 1.0f, 1.0f, 1.0f };
    Mesh* plane = new Mesh("../resources/models/plane.obj", planeColour);
    plane->isRigidBody = true;

    Vector3f flagPoleColour = { 0.337f, 0.184f, 0.054f };
    Mesh* flagPole = new Mesh("../resources/models/flagPole.obj", flagPoleColour);
    Mesh* flagPole2 = new Mesh("../resources/models/flagPole2.obj", flagPoleColour);

    Vector3f flagColour = { 0.6f, 0.0f, 0.0f };
    Mesh* flag = new Mesh("../resources/models/flag.obj", flagColour);
    flag->gravityAffected = true;
    flag->windAffected = true;
    Mesh* flagHigh = new Mesh("../resources/models/flagHigh.obj", flagColour);
    flagHigh->gravityAffected = true;
    flagHigh->windAffected = true;

    // Setup constraints
    testCube->constraints.push_back(buildFixedConstraint(testCube, 3, testCube->initialVertices[3]));
    buildEdgeConstraints(testCube);

    for (int i = 0; i < 7; i++) {
        flag->constraints.push_back(buildFixedConstraint(flag, i, flag->initialVertices[i]));
    }
    buildEdgeConstraints(flag);
    buildBendConstraints(flag);

    for (int i = 0; i < 14; i++) {
        flagHigh->constraints.push_back(buildFixedConstraint(flagHigh, i, flagHigh->initialVertices[i]));
    }
    buildEdgeConstraints(flagHigh);
    buildBendConstraints(flagHigh);

    configurationA->staticObjects.push_back(plane);
    configurationA->staticObjects.push_back(flagPole);
    configurationA->staticObjects.push_back(flagPole2);
    configurationA->simulatedObjects.push_back(testCube);
    configurationA->simulatedObjects.push_back(flag);
    configurationA->simulatedObjects.push_back(flagHigh);

    testCube->applyImpulse(Vector3f(1.0f, 2.5f, -0.5f));
    testCube->vertices[0] += Vector3f(1.50f, 0.0f, 0.0f);
}

void Scene::setupConfigurationB() {
    configurationB = new Configuration();

    Vector3f planeColour = { 1.0f, 1.0f, 1.0f };
    Mesh* plane = new Mesh("../resources/models/plane.obj", planeColour);
    plane->isRigidBody = true;

    Vector3f clothColour = { 0.0f, 0.6f, 0.0f };
    Mesh* cloth = new Mesh("../resources/models/cloth.obj", clothColour);
    cloth->gravityAffected = true;

    Vector3f passiveCubeColour = { 0.5f, 0.5f, 0.5f };
    Mesh* passiveCube = new Mesh("../resources/models/passiveCube.obj", planeColour);
    passiveCube->isRigidBody = true;

    buildEdgeConstraints(cloth);
    buildBendConstraints(cloth);

    configurationB->staticObjects.push_back(plane);
    configurationB->simulatedObjects.push_back(cloth);
    configurationB->staticObjects.push_back(passiveCube);
}

void Scene::setupConfigurationC() {
    configurationC = new Configuration();

    Vector3f planeColour = { 1.0f, 1.0f, 1.0f };
    Mesh* plane = new Mesh("../resources/models/plane.obj", planeColour);
    plane->isRigidBody = true;

//    Mesh* simple = new Mesh("../resources/models/simple.obj", planeColour);
//    Mesh* simple = new Mesh("../resources/models/selfIntersectionTest.obj", planeColour);
//    simple->gravityAffected = true;

//    simple->constraints.push_back(buildFixedConstraint(simple, 0, simple->initialVertices[0]));
//    simple->constraints.push_back(buildFixedConstraint(simple, 1, simple->initialVertices[1]));
//    simple->constraints.push_back(buildFixedConstraint(simple, 2, simple->initialVertices[2]));
//    simple->constraints.push_back(buildFixedConstraint(simple, 3, simple->initialVertices[3]));
//    buildEdgeConstraints(simple);

    configurationC->staticObjects.push_back(plane);
//    configurationC->simulatedObjects.push_back(simple);
}
