//
// Created by Jack Purvis
//

#include <iostream>
#include <main.hpp>
#include <constraint.hpp>
#include <scene.hpp>

Scene::Scene() {

    // Setup camera
    camera = new Camera();

    // Setup scene configurations
    setupConfigurationA();
    setupConfigurationB();
    setupConfigurationC();
    setupConfigurationD();
    currentConfiguration = configurationA;
}

Scene::~Scene() {
    delete camera;
    delete configurationA;
    delete configurationB;
    delete configurationC;
    delete configurationD;
}

void Scene::reset() {
    for (Mesh* mesh : currentConfiguration->simulatedObjects) {
        mesh->reset();
    }

    currentConfiguration->estimatePositions.resize(currentConfiguration->estimatePositions.size(), Vector3f::Zero());
}

void Scene::setConfiguration(int index) {
    switch (index) {
        case 0:
            currentConfiguration = configurationA;
            break;
        case 1:
            currentConfiguration = configurationB;
            break;
        case 2:
            currentConfiguration = configurationC;
            break;
        case 3:
            currentConfiguration = configurationD;
            break;
        default:
            currentConfiguration = configurationA;
            break;
    }
}

void Scene::translateInteraction(Vector3f translate) {

    // Translate the attachment points in scene 3
    if (currentConfiguration == configurationC) {
        configurationC->simulatedObjects[0]->translate(translate);
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

    for (Mesh* mesh : currentConfiguration->staticObjects) {
        mesh->render(camera, modelMatrix);
    }

    for (Mesh* mesh : currentConfiguration->simulatedObjects) {
        mesh->render(camera, modelMatrix);
    }
}

void Scene::setupConfigurationA() {
    configurationA = new Configuration();

    addPlaneToConfiguration(configurationA);

    Vector3f flagPoleColour = { 0.337f, 0.184f, 0.054f };
    Mesh* flagPole = new Mesh("../resources/models/sceneA/flagPole.obj", flagPoleColour);
    Mesh* flagPole2 = new Mesh("../resources/models/sceneA/flagPole2.obj", flagPoleColour);

    Vector3f flagColour = { 0.6f, 0.0f, 0.0f };
    Mesh* flag = new Mesh("../resources/models/sceneA/flag.obj", flagColour);
    flag->gravityAffected = true;
    flag->windAffected = true;
    Mesh* flagHigh = new Mesh("../resources/models/sceneA/flagHigh.obj", flagColour);
    flagHigh->gravityAffected = true;
    flagHigh->windAffected = true;

    configurationA->staticObjects.push_back(flagPole);
    configurationA->staticObjects.push_back(flagPole2);
    configurationA->simulatedObjects.push_back(flag);
    configurationA->simulatedObjects.push_back(flagHigh);

    setupEstimatePositionOffsets(configurationA);

    for (int i = 0; i < 7; i++) buildFixedConstraint(configurationA, flag, i, flag->initialVertices[i]);
    buildEdgeConstraints(configurationA, flag);
    buildBendConstraints(configurationA, flag);

    for (int i = 0; i < 14; i++) buildFixedConstraint(configurationA, flagHigh, i, flagHigh->initialVertices[i]);
    buildEdgeConstraints(configurationA, flagHigh);
    buildBendConstraints(configurationA, flagHigh);
}

void Scene::setupConfigurationB() {
    configurationB = new Configuration();

    addPlaneToConfiguration(configurationB);

    Vector3f clothColour = { 0.0f, 0.6f, 0.0f };
    Mesh* cloth = new Mesh("../resources/models/sceneB/cloth.obj", clothColour);
    cloth->gravityAffected = true;

    Vector3f resetObjectColour = { 0.5f, 0.5f, 0.5f };
    Mesh* restObject = new Mesh("../resources/models/sceneB/sphere.obj", resetObjectColour);
    restObject->isRigidBody = true;

    configurationB->simulatedObjects.push_back(cloth);
    configurationB->staticObjects.push_back(restObject);

    setupEstimatePositionOffsets(configurationB);

    buildEdgeConstraints(configurationB, cloth);
    buildBendConstraints(configurationB, cloth);
}

void Scene::setupConfigurationC() {
    configurationC = new Configuration();

    addPlaneToConfiguration(configurationC);

    Vector3f solidColour = { 1.0f, 1.0f, 1.0f };
    Mesh* attachPoints = new Mesh("../resources/models/sceneC/attachPoints.obj", solidColour, 0.0f);
    attachPoints->isRigidBody = true;

    Vector3f clothColour = { 0.8f, 0.4f, 0.1f };
    Mesh* cloth = new Mesh("../resources/models/sceneC/cloth.obj", clothColour);
    cloth->gravityAffected = true;
    cloth->windAffected = true;

    Mesh* bar = new Mesh("../resources/models/sceneC/bar.obj", solidColour, 0.5f);
    bar->isRigidBody = true;
    bar->gravityAffected = true;
    bar->windAffected = true;

    configurationC->simulatedObjects.push_back(attachPoints);
    configurationC->simulatedObjects.push_back(cloth);
    configurationC->simulatedObjects.push_back(bar);

    setupEstimatePositionOffsets(configurationC);

    buildEdgeConstraints(configurationC, cloth);
    buildBendConstraints(configurationC, cloth);

    buildRigidBodyConstraints(configurationC, bar);

    buildTwoWayCouplingConstraints(configurationC, cloth);
}

void Scene::setupConfigurationD() {
    configurationD = new Configuration();

    addPlaneToConfiguration(configurationD);

    Vector3f colourA = { 1.0f, 1.0f, 0.0f };
    Vector3f colourB = { 1.0f, 0.0f, 1.0f };
    Vector3f colourC = { 0.0f, 1.0f, 1.0f };
    Vector3f colourD = { 0.4f, 0.4f, 0.4f };

    Mesh* cube = new Mesh("../resources/models/sceneD/cube.obj", colourA);
    cube->isRigidBody = true;
    cube->gravityAffected = true;

    Mesh* pyramid = new Mesh("../resources/models/sceneD/pyramid.obj", colourB);
    pyramid->isRigidBody = true;
    pyramid->gravityAffected = true;

    Mesh* cylinder = new Mesh("../resources/models/sceneD/cylinder.obj", colourC);
    cylinder->isRigidBody = true;
    cylinder->gravityAffected = true;

    Mesh* sphere = new Mesh("../resources/models/sceneD/sphere.obj", colourD);
    sphere->isRigidBody = true;
    sphere->gravityAffected = true;

    configurationD->simulatedObjects.push_back(cube);
    configurationD->simulatedObjects.push_back(pyramid);
    configurationD->simulatedObjects.push_back(cylinder);
    configurationD->simulatedObjects.push_back(sphere);

    setupEstimatePositionOffsets(configurationD);

    buildRigidBodyConstraints(configurationD, cube);
    buildRigidBodyConstraints(configurationD, pyramid);
    buildRigidBodyConstraints(configurationD, cylinder);
    buildRigidBodyConstraints(configurationD, sphere);
}

void Scene::addPlaneToConfiguration(Configuration* configuration) {
    Vector3f planeColour = { 1.0f, 1.0f, 1.0f };
    Mesh* plane = new Mesh("../resources/models/plane.obj", planeColour);
    plane->isRigidBody = true;

    configuration->staticObjects.push_back(plane);
}

void Scene::setupEstimatePositionOffsets(Configuration* configuration) {
    int totalNumVertices = 0;

    for (Mesh* mesh : configuration->simulatedObjects) {
        mesh->estimatePositionsOffset = totalNumVertices;
        totalNumVertices += mesh->numVertices;

        for (int i = 0; i < mesh->numVertices; i++) configuration->inverseMasses.push_back(mesh->inverseMass);
    }

    configuration->estimatePositions.resize((size_t) totalNumVertices, Vector3f::Zero());
}
