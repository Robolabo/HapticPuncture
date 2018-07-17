//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   3.1.1 $Rev: 1907 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "chai3d.h"
#include "GEL3D.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <iostream>
#include <fstream>


int fd, n, i;
int buf[1];
struct termios toptions;

//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------
#ifndef MACOSX
#include "GL/glut.h"
#else
#include "GLUT/glut.h"
#endif
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled 
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;


//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// two light sources to illuminate the objects in the world
cDirectionalLight *light1;
cDirectionalLight *light2;

// force scale factor
double deviceForceScale;

// scale factor between the device workspace and cursor workspace
double workspaceScaleFactor;

// desired workspace radius of the virtual cursor
double cursorWorkspaceRadius;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelHapticRate;

// a global variable to store the position [m] of the haptic device
cVector3d hapticDevicePosition;

// a label to display the position of the haptic device
cLabel* labelHapticDevicePosition;

// final number of nodes
int numNodesTot = 0;

// variable to scale the models (from meters to centimeters)
double scale = 0.01;

// a global variable to store the position of the meshes
cVector3d generalPos(0.0, 0.0, 0.0);//(1.0, 0.0, 1.5);

// a global variable to store the rotation of the meshes
cVector3d generalRot;
int degrees;

// select meshes to display
bool addVertebra1 = true;
bool addVertebra2 = true;
bool addDisco = true;
bool addInterspinous = true;
bool addSupraspinous = true;
bool addFlavum = true;

// flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;

// frequency counter to measure the simulation haptic rate
cFrequencyCounter frequencyCounter;

// information about computer screen and GLUT display window
int screenW;
int screenH;
int windowW;
int windowH;
int windowPosX;
int windowPosY;

// root resource path
string resourceRoot;

// save forces in a txt file
bool saveForces = true;

// txt file to save the resulting forces
ofstream outputFile("./force_vertebra.txt");

//------------------------------------------------------------------------------
// GEL
//------------------------------------------------------------------------------

// deformable world
cGELWorld* defWorld;

// lumbar vertebra 2 mesh
cGELMesh* defVertebra1;

// lumbar vertebra 3 mesh
cGELMesh* defVertebra2;

// inververtebral disk mesh
cGELMesh* defDisco;

// interspinous ligament mesh
cGELMesh* defInterspinous;

// supraspinous ligament mesh
cGELMesh* defSupraspinous;

// flavum ligament mesh
cGELMesh* defFlavum;

// conjuntivo mesh
cGELMesh* defConjuntivo;

// haptic device model
cShapeCylinder* device;
double deviceRadius;

// radius of the dynamic model sphere (GEM)
double radius;

// stiffness properties between the haptic device tool and the model (GEM)
double stiffness;


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback to render graphic scene
void updateGraphics(void);

// callback of GLUT timer
void graphicsTimer(int data);

// function that closes the application
void close(void);

// main haptics simulation loop
void updateHaptics(void);

// compute forces between tool and environment
cVector3d computeForce(const cVector3d& a_cursor,
                       double a_cursorRadius,
                       const cVector3d& a_spherePos,
                       double a_radius,
                       double a_stiffness);


//------------------------------------------------------------------------------
// DECLARED MACROS
//------------------------------------------------------------------------------

// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())


//==============================================================================
/*
    HapticPuncture.cpp

    This is an open-source, virtual reality based simulator for training in epi-
    dural procedures. It is based in the GEL module given by CHAI3D to simulate 
    the different tissues involved in the puncture: 
	- Two vertebrae from the lumbar zone (in this case L2 and L3) 
	- The intervertebral disk
	- The intraspinous ligament
	- The supraspinous ligament
	- The flavum ligament
	- The muscle
	- The skin
    The meshes modelling each one of these tissues are loaded and simulate defo-
    rmable objects. Interacting with a custom haptic device, forces are created.
*/
//==============================================================================

int main(int argc, char* argv[])
{
    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    cout << endl;
    cout << "|===================================|" << endl;
    cout << "|         HapticPuncture            |" << endl;
    cout << "|===================================|" << endl;
    cout << "|                                   |" << endl;
    cout << "| Epidural anesthesia simulation.   |" << endl;
    cout << "| Based on CHAI3D GEL implementation|" << endl;
    cout << "|-----------------------------------|" << endl << endl << endl;
    cout << endl << endl;

    // parse first arg to try and locate resources
    resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);

    //-----------------------------------------------------------------------
    // SERIAL PORT SETTINGS
    //-----------------------------------------------------------------------

    /* open serial port */
    fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
    if (fd == -1){
        cout << "Haptic device communication failed" << endl;
    }
    else{
        printf("Haptic device communication in port %i\n", fd);
    }

    /* wait for the Arduino to reboot */
    usleep(3500000);

    /* get current serial port settings */
    tcgetattr(fd, &toptions);
    /* set 115200 baud both ways */
    cfsetispeed(&toptions, B115200);
    cfsetospeed(&toptions, B115200);
    /* 8 bits, no parity, no stop bits */
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    /* commit the serial port settings */
    tcsetattr(fd, TCSANOW, &toptions);

    //--------------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLUT
    glutInit(&argc, argv);

    // retrieve  resolution of computer display and position window accordingly
    screenW = glutGet(GLUT_SCREEN_WIDTH);
    screenH = glutGet(GLUT_SCREEN_HEIGHT);
    windowW = 0.8 * screenH;
    windowH = 0.5 * screenH;
    windowPosY = (screenH - windowH) / 2;
    windowPosX = windowPosY; 

    // initialize the OpenGL GLUT window
    glutInitWindowPosition(windowPosX, windowPosY);
    glutInitWindowSize(windowW, windowH);

    if (stereoMode == C_STEREO_ACTIVE)
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE | GLUT_STEREO);
    else
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);

    // create display context and initialize GLEW library
    glutCreateWindow(argv[0]);

#ifdef GLEW_VERSION
    // initialize GLEW
    glewInit();
#endif

    // setup GLUT options
    glutDisplayFunc(updateGraphics);
    glutSetWindowTitle("HapticPuncture");

    // set fullscreen mode
    if (fullscreen)
    {
        glutFullScreen();
    }


    //--------------------------------------------------------------------------
    // 3D - SCENEGRAPH
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // set background color
    world->m_backgroundColor.setBlack();

    // position and orient the camera
    camera->set(cVector3d(-0.1, 0.2, 0.0),    // camera position (eye) - From the side
		//cVector3d(0.2, 0.0, 0.0)    // OR camera position (eye) - From the front
                cVector3d(0.0, 0.0, 0.0),     // lookat position (target)
                cVector3d(0.0, 0.0, 1.0));    // direction of the (up) vector

    // set the near and far clipping planes of the camera
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.02);
    camera->setStereoFocalLength(3.0);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // enable multi-pass rendering to handle transparent objects
    camera->setUseMultipassTransparency(true);

    // create a first directional light source
    light1 = new cDirectionalLight(world);

    // insert light source inside world
    world->addChild(light1);

    // enable light source
    light1->setEnabled(true);                   

    // define direction of light beam
    light1->setDir(-1.0,-1.0, 0.0);

    // create a second directional light source
    light2 = new cDirectionalLight(world);

    // insert light source inside world
    world->addChild(light2);

    // enable light source
    light2->setEnabled(true);                   

    // define direction of light beam
    light2->setDir(-1.0, 1.0, 0.0); 


    //--------------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //--------------------------------------------------------------------------
 
    // desired workspace radius of the cursor
    cursorWorkspaceRadius = 1;

    // scale factor between the physical workspace of the haptic device and the 
    // virtual workspace
    workspaceScaleFactor = 0.0045; //adjusted manually

    // define a scale factor between the force perceived at the cursor and the
    // forces actually sent to the haptic device
    deviceForceScale = 1;

    // create a cylinder to represent the epidural catheter
    deviceRadius = 0.0018; // [1.8mm ø]
    device = new cShapeCylinder(deviceRadius, deviceRadius, 0.03); // [3cm long]
    world->addChild(device);
    
    // adapt the needle to the environment
    device->rotateAboutGlobalAxisDeg(0.0, 1.0, 0.0, 90);

    // define a texture for the needle
    device->m_material->m_ambient.set(1.0f, 0.4f, 0.4f, 0.5f);
    device->m_material->m_diffuse.set(1.0f, 0.7f, 0.7f, 0.5f);
    device->m_material->m_specular.set(1.0f, 1.0f, 1.0f, 0.5f);
    device->m_material->setShininess(100);

    // interaction stiffness between tool and deformable model 
    stiffness = 100;
    

    //--------------------------------------------------------------------------
    // COMPOSE THE VIRTUAL SCENE
    //--------------------------------------------------------------------------

    //==========================================================================
    // GENERAL SETINGS
    //==========================================================================

    // create a world which supports deformable object
    defWorld = new cGELWorld();
    world->addChild(defWorld);
    
    // define 3 factors to reduce the number of nodes of the different models
    int factorVert = 20;  // for the vertebrae
    int factorDisk = 3;   // for the intervertebral disk
    int factor = 1;	  // for the soft tissues (except from the disk)

    //==========================================================================
    // 1- VERTEBRA1 - L2 VERTEBRA (SUPERIOR)
    //==========================================================================

    if (addVertebra1){
        // create a deformable mesh
        defVertebra1 = new cGELMesh();
        defWorld->m_gelMeshes.push_front(defVertebra1);

	// position the mesh
        defVertebra1->setLocalPos(generalPos);

	// load the model
        bool fileload1;
	fileload1 = defVertebra1->loadFromFile(RESOURCE_PATH("../resources/models/vertebra_L2/L2.stl"));
        if (!fileload1)
        {
            #if defined(_MSVC)
	    fileload1 = defVertebra1->loadFromFile("../../../bin/resources/models/vertebra_L2/L2.stl");
            #endif
        }
        if (!fileload1)
        {
            printf("Error - Vertebra1 3D Model failed to load correctly.\n");
            close();
            return (-1);
        }

        // resize model (from meters to centimeters)
        defVertebra1->scale(scale);

        // set some material color on the object
        cMaterial mat1;
        mat1.setWhite();
        mat1.setShininess(100);
        defVertebra1->setMaterial(mat1, true);

        // let's create a some environment mapping
        shared_ptr<cTexture2d> texture1(new cTexture2d());
        fileload1 = texture1->loadFromFile(RESOURCE_PATH("../resources/images/hueso.jpg"));
        if (!fileload1)
        {
            #if defined(_MSVC)
            fileload1 = texture1->loadFromFile("../../../bin/resources/images/hueso.jpg");
            #endif
        }
        if (!fileload1)
        {
            cout << "Error - Vertebra1 texture failed to load correctly." << endl;
            close();
            return (-1);
        }

        // enable environmental texturing
        texture1->setEnvironmentMode(GL_DECAL);
        texture1->setSphericalMappingEnabled(true);

        // assign and enable texturing
        defVertebra1->setTexture(texture1, true);
        defVertebra1->setUseTexture(true, true);

        // set object to be transparent
        defVertebra1->setTransparencyLevel(0.45, true, true);

        // build dynamic vertices
        defVertebra1->buildVertices();

        // set default properties for skeleton nodes
        cGELSkeletonNode::s_default_radius        = 0.002;  // [m]
        cGELSkeletonNode::s_default_kDampingPos   = 1500.0;
        cGELSkeletonNode::s_default_kDampingRot   = 0.6;
        cGELSkeletonNode::s_default_mass          = 0.001; // [kg]
        cGELSkeletonNode::s_default_showFrame     = true;
        cGELSkeletonNode::s_default_color.setBlueCornflower();
        cGELSkeletonNode::s_default_useGravity    = false;
        cGELSkeletonNode::s_default_gravity.set(0.00, 0.00,-9.81);
        radius = cGELSkeletonNode::s_default_radius;

        // use internal skeleton as deformable model
        defVertebra1->m_useSkeletonModel = true;

        //------------Create Nodes------------------------------------------------//
        // get number of vertices
        int numVertices1 = defVertebra1->getNumVertices();
        int numNodes1 = int(numVertices1/(factorVert));
	numNodesTot = numNodesTot + numNodes1;
        cout << "Superior vertebra number of nodes: " << endl;
        cout << numNodes1 << endl;

        // create nodes array
        cGELSkeletonNode* nodes1[numNodes1];

        // create an array of nodes
        for (int i=0; i<numNodes1; i++)
        {
            cGELSkeletonNode* newNode = new cGELSkeletonNode();
            nodes1[i] = newNode;
            defVertebra1->m_nodes.push_front(newNode);

            // get current deformable vertex
            cGELVertex* curVertex = &defVertebra1->m_gelVertices[i*factorVert];

            // get current vertex position
            cVector3d posVertex = curVertex->m_mesh->m_vertices->getLocalPos(curVertex->m_vertexIndex);
            double posx = posVertex(0);
            double posy = posVertex(1);
            double posz = posVertex(2);
            newNode->m_pos.set(posx, posy, posz);

        }


        //------------------------------------------------------------------------//

        // set corner nodes as fixed
        // 50% of nodes will be fixed
        int fixedNodes1 = int(numNodes1*1);
        for (int i=0; i<fixedNodes1-1; i++){

            // All nodes fixed (100%)
            nodes1[i*1]->m_fixed = true;

        }

        // set default physical properties for links
        cGELSkeletonLink::s_default_kSpringElongation = 500.0;  // [N/m]
        cGELSkeletonLink::s_default_kSpringFlexion    = 0.5;   // [Nm/RAD]
        cGELSkeletonLink::s_default_kSpringTorsion    = 0.1;   // [Nm/RAD]
        cGELSkeletonLink::s_default_color.setBlueCornflower();


        //------------Create Links between Nodes----------------------------------//
        for (int i=0; i<numNodes1-1; i++)
        {
            // cGELSkeletonLink* newLink = new cGELSkeletonLink(nodes[i], nodes[i+1]); 
            // defMuscle->m_links.push_front(newLink);

            // get current deformable vertex
            cGELVertex* curVertex = &defVertebra1->m_gelVertices[i*factorVert]; //factor = 300

            // get current vertex position
            cVector3d pos = curVertex->m_mesh->m_vertices->getLocalPos(curVertex->m_vertexIndex);

            // initialize constant
            double min_distance = 99999999999999999.0;
            cGELSkeletonNode* nearest_node = NULL;
            cGELSkeletonNode* almost_nearest_node = NULL;

            // search for the 2 nearest nodes
            list<cGELSkeletonNode*>::iterator itr;
            for(itr = defVertebra1->m_nodes.begin(); itr != defVertebra1->m_nodes.end(); ++itr)
            {
                cGELSkeletonNode* nextNode = *itr;
                double distance = cDistance(pos, nextNode->m_pos);
                if (distance < min_distance)
                {
                    min_distance = distance;
                    nearest_node = nextNode;
                    almost_nearest_node = nearest_node;
                }
            }

            // create a link with the two nearest nodes of the mesh
            cGELSkeletonLink* nearest_link = new cGELSkeletonLink(nodes1[i], nearest_node);
            cGELSkeletonLink* almost_nearest_link = new cGELSkeletonLink(nodes1[i], almost_nearest_node);
            defVertebra1->m_links.push_front(nearest_link);
            defVertebra1->m_links.push_front(almost_nearest_link);        
        }
        //------------------------------------------------------------------------//

        // connect skin (mesh) to skeleton (GEM). if __false__ then skin mesh is connected to nodes and links.
        defVertebra1->connectVerticesToSkeleton(false);

        // show/hide underlying dynamic skeleton model
        defVertebra1->m_showSkeletonModel = false;
    }
    
    //==========================================================================
    // 2- VERTEBRA2 - L3 VERTEBRA (INFERIOR)
    //==========================================================================

    if (addVertebra2){
        // create a deformable mesh
        defVertebra2 = new cGELMesh();
        defWorld->m_gelMeshes.push_front(defVertebra2);

	// position the mesh
        defVertebra2->setLocalPos(generalPos);

	// load the model
        bool fileload2;
	fileload2 = defVertebra2->loadFromFile(RESOURCE_PATH("../resources/models/vertebra_L3/L3.stl"));
        if (!fileload2)
        {
            #if defined(_MSVC)
	    fileload2 = defVertebra2->loadFromFile("../../../bin/resources/models/vertebra_L3/L3.stl");
            #endif
        }
        if (!fileload2)
        {
            printf("Error - Vertebra2 3D Model failed to load correctly.\n");
            close();
            return (-1);
        }

        // resize model (from meters to centimeters)
        defVertebra2->scale(scale);

        // set some material color on the object
        cMaterial mat2;
        mat2.setWhite();
        mat2.setShininess(100);
        defVertebra2->setMaterial(mat2, true);

        // let's create a some environment mapping
        shared_ptr<cTexture2d> texture2(new cTexture2d());
        fileload2 = texture2->loadFromFile(RESOURCE_PATH("../resources/images/hueso.jpg"));
        if (!fileload2)
        {
            #if defined(_MSVC)
            fileload2 = texture2->loadFromFile("../../../bin/resources/images/hueso.jpg");
            #endif
        }
        if (!fileload2)
        {
            cout << "Error - Vertebra2 texture failed to load correctly." << endl;
            close();
            return (-1);
        }

        // enable environmental texturing
        texture2->setEnvironmentMode(GL_DECAL);
        texture2->setSphericalMappingEnabled(true);

        // assign and enable texturing
        defVertebra2->setTexture(texture2, true);
        defVertebra2->setUseTexture(true, true);

        // set object to be transparent
        defVertebra2->setTransparencyLevel(0.45, true, true);

        // build dynamic vertices
        defVertebra2->buildVertices();

        // set default properties for skeleton nodes
        cGELSkeletonNode::s_default_radius        = 0.002;  // [m]
        cGELSkeletonNode::s_default_kDampingPos   = 1500.0;
        cGELSkeletonNode::s_default_kDampingRot   = 0.6;
        cGELSkeletonNode::s_default_mass          = 0.001; // [kg]
        cGELSkeletonNode::s_default_showFrame     = true;
        cGELSkeletonNode::s_default_color.setBlueCornflower();
        cGELSkeletonNode::s_default_useGravity    = false;
        cGELSkeletonNode::s_default_gravity.set(0.00, 0.00,-9.81);
        radius = cGELSkeletonNode::s_default_radius;


        // use internal skeleton as deformable model
        defVertebra2->m_useSkeletonModel = true;

        //------------Create Nodes------------------------------------------------//
        // get number of vertices
        int numVertices2 = defVertebra2->getNumVertices();
        int numNodes2 = int(numVertices2/(factorVert));
	numNodesTot = numNodesTot + numNodes2;
        cout << "Inferior vertebra number of nodes: " << endl;
        cout << numNodes2 << endl;

        // create nodes array
        cGELSkeletonNode* nodes2[numNodes2];

        // create an array of nodes
        for (int i=0; i<numNodes2; i++)
        {
            cGELSkeletonNode* newNode = new cGELSkeletonNode();
            nodes2[i] = newNode;
            defVertebra2->m_nodes.push_front(newNode);

            // get current deformable vertex
            cGELVertex* curVertex = &defVertebra2->m_gelVertices[i*factorVert];

            // get current vertex position
            cVector3d posVertex = curVertex->m_mesh->m_vertices->getLocalPos(curVertex->m_vertexIndex);
            double posx = posVertex(0);
            double posy = posVertex(1);
            double posz = posVertex(2);
            newNode->m_pos.set(posx, posy, posz);

        }


        //------------------------------------------------------------------------//

        // set corner nodes as fixed
        // 100% of nodes will be fixed
        int fixedNodes2 = int(numNodes2*1);
        for (int i=0; i<fixedNodes2-1; i++){

            // All nodes fixed (100%)
            nodes2[i*1]->m_fixed = true;

        }

        // set default physical properties for links
        cGELSkeletonLink::s_default_kSpringElongation = 500.0;  // [N/m]
        cGELSkeletonLink::s_default_kSpringFlexion    = 0.5;   // [Nm/RAD]
        cGELSkeletonLink::s_default_kSpringTorsion    = 0.1;   // [Nm/RAD]
        cGELSkeletonLink::s_default_color.setBlueCornflower();


        //------------Create Links between Nodes----------------------------------//
        for (int i=0; i<numNodes2-1; i++)
        {
            // cGELSkeletonLink* newLink = new cGELSkeletonLink(nodes[i], nodes[i+1]); 
            // defMuscle->m_links.push_front(newLink);

            // get current deformable vertex
            cGELVertex* curVertex = &defVertebra2->m_gelVertices[i*factorVert];

            // get current vertex position
            cVector3d pos = curVertex->m_mesh->m_vertices->getLocalPos(curVertex->m_vertexIndex);

            // initialize constant
            double min_distance = 99999999999999999.0;
            cGELSkeletonNode* nearest_node = NULL;
            cGELSkeletonNode* almost_nearest_node = NULL;

            // search for the 2 nearest nodes
            list<cGELSkeletonNode*>::iterator itr;
            for(itr = defVertebra2->m_nodes.begin(); itr != defVertebra2->m_nodes.end(); ++itr)
            {
                cGELSkeletonNode* nextNode = *itr;
                double distance = cDistance(pos, nextNode->m_pos);
                if (distance < min_distance)
                {
                    min_distance = distance;
                    nearest_node = nextNode;
                    almost_nearest_node = nearest_node;
                }
            }

            // create a link with the two nearest nodes of the mesh
            cGELSkeletonLink* nearest_link = new cGELSkeletonLink(nodes2[i], nearest_node);
            cGELSkeletonLink* almost_nearest_link = new cGELSkeletonLink(nodes2[i], almost_nearest_node);
            defVertebra2->m_links.push_front(nearest_link);
            defVertebra2->m_links.push_front(almost_nearest_link);        
        }
        //------------------------------------------------------------------------//

        // connect skin (mesh) to skeleton (GEM). if __false__ then skin mesh is connected to nodes and links.
        defVertebra2->connectVerticesToSkeleton(false);

        // show/hide underlying dynamic skeleton model
        defVertebra2->m_showSkeletonModel = false;
    }
    
    //==========================================================================
    // 3 - DISK
    //==========================================================================

    if(addDisco){
   // create a deformable mesh
        defDisco = new cGELMesh();
        defWorld->m_gelMeshes.push_front(defDisco);

	// position the mesh
        defDisco->setLocalPos(generalPos);

	// load the model
        bool fileload3;
	fileload3 = defDisco->loadFromFile(RESOURCE_PATH("../resources/models/disk/disk.stl"));
        if (!fileload3)
        {
            #if defined(_MSVC)
	    fileload3 = defDisco->loadFromFile("../../../bin/resources/models/disk/disk.stl");
            #endif
        }
        if (!fileload3)
        {
            printf("Error - Disco 3D Model failed to load correctly.\n");
            close();
            return (-1);
        }

        // resize model (from meters to centimeters)
        defDisco->scale(scale);

        // set some material color on the object
        cMaterial mat3;
        mat3.setWhite();
        mat3.setShininess(100);
        defDisco->setMaterial(mat3, true);

        // let's create a some environment mapping
        shared_ptr<cTexture2d> texture3(new cTexture2d());
        fileload3 = texture3->loadFromFile(RESOURCE_PATH("../resources/images/conjuntivo.jpg"));
        if (!fileload3)
        {
            #if defined(_MSVC)
            fileload3 = texture3->loadFromFile("../../../bin/resources/images/conjuntivo.jpg");
            #endif
        }
        if (!fileload3)
        {
            cout << "Error - Disco texture failed to load correctly." << endl;
            close();
            return (-1);
        }

        // enable environmental texturing
        texture3->setEnvironmentMode(GL_DECAL);
        texture3->setSphericalMappingEnabled(true);

        // assign and enable texturing
        defDisco->setTexture(texture3, true);
        defDisco->setUseTexture(true, true);

        // set object to be transparent
        defDisco->setTransparencyLevel(0.45, true, true);

        // build dynamic vertices
        defDisco->buildVertices();

        // set default properties for skeleton nodes
        cGELSkeletonNode::s_default_radius        = 0.002;  // [m]
        cGELSkeletonNode::s_default_kDampingPos   = 50.0;
        cGELSkeletonNode::s_default_kDampingRot   = 0.6;
        cGELSkeletonNode::s_default_mass          = 0.001; // [kg]
        cGELSkeletonNode::s_default_showFrame     = true;
        cGELSkeletonNode::s_default_color.setBlueCornflower();
        cGELSkeletonNode::s_default_useGravity    = false;
        cGELSkeletonNode::s_default_gravity.set(0.00, 0.00,-9.81);
        radius = cGELSkeletonNode::s_default_radius;

        // use internal skeleton as deformable model
        defDisco->m_useSkeletonModel = true;

        //------------Create Nodes------------------------------------------------//
        // get number of vertices
        int numVertices3 = defDisco->getNumVertices();
        int numNodes3 = int(numVertices3/factorDisk);
	numNodesTot = numNodesTot + numNodes3;
        cout << "Intervertebral disk number of nodes: " << endl;
        cout << numNodes3 << endl;

        // create nodes array
        cGELSkeletonNode* nodes3[numNodes3];

        // create an array of nodes
        for (int i=0; i<numNodes3; i++)
        {
            cGELSkeletonNode* newNode = new cGELSkeletonNode();
            nodes3[i] = newNode;
            defDisco->m_nodes.push_front(newNode);

            // get current deformable vertex
            cGELVertex* curVertex = &defDisco->m_gelVertices[i*factorDisk];

            // get current vertex position
            cVector3d posVertex = curVertex->m_mesh->m_vertices->getLocalPos(curVertex->m_vertexIndex);
            double posx = posVertex(0);
            double posy = posVertex(1);
            double posz = posVertex(2);
            newNode->m_pos.set(posx, posy, posz);

        }


        //------------------------------------------------------------------------//

        // set corner nodes as fixed
        // 5% of nodes will be fixed
        int fixedNodes3 = int(numNodes3*0.05);
        for (int i=0; i<fixedNodes3-1; i++){

            // one node fixed every 20 (5%)
            nodes3[i*20]->m_fixed = true;

        }

        // set default physical properties for links
        cGELSkeletonLink::s_default_kSpringElongation = 300.0;  // [N/m]
        cGELSkeletonLink::s_default_kSpringFlexion    = 0.5;   // [Nm/RAD]
        cGELSkeletonLink::s_default_kSpringTorsion    = 0.1;   // [Nm/RAD]
        cGELSkeletonLink::s_default_color.setBlueCornflower();


        //------------Create Links between Nodes----------------------------------//
        for (int i=0; i<numNodes3-1; i++)
        {
            // cGELSkeletonLink* newLink = new cGELSkeletonLink(nodes[i], nodes[i+1]); 
            // defMuscle->m_links.push_front(newLink);

            // get current deformable vertex
            cGELVertex* curVertex = &defDisco->m_gelVertices[i*factorDisk];

            // get current vertex position
            cVector3d pos = curVertex->m_mesh->m_vertices->getLocalPos(curVertex->m_vertexIndex);

            // initialize constant
            double min_distance = 99999999999999999.0;
            cGELSkeletonNode* nearest_node = NULL;
            cGELSkeletonNode* almost_nearest_node = NULL;

            // search for the 2 nearest nodes
            list<cGELSkeletonNode*>::iterator itr;
            for(itr = defDisco->m_nodes.begin(); itr != defDisco->m_nodes.end(); ++itr)
            {
                cGELSkeletonNode* nextNode = *itr;
                double distance = cDistance(pos, nextNode->m_pos);
                if (distance < min_distance)
                {
                    min_distance = distance;
                    nearest_node = nextNode;
                    almost_nearest_node = nearest_node;
                }
            }

            // create a link with the two nearest nodes of the mesh
            cGELSkeletonLink* nearest_link = new cGELSkeletonLink(nodes3[i], nearest_node);
            cGELSkeletonLink* almost_nearest_link = new cGELSkeletonLink(nodes3[i], almost_nearest_node);
            defDisco->m_links.push_front(nearest_link);
            defDisco->m_links.push_front(almost_nearest_link);        
        }
        //------------------------------------------------------------------------//

        // connect skin (mesh) to skeleton (GEM). if __false__ then skin mesh is connected to nodes and links.
        defDisco->connectVerticesToSkeleton(false);

        // show/hide underlying dynamic skeleton model
        defDisco->m_showSkeletonModel = false;
    }
    
    
    //==========================================================================
    // 4 - INTERSPINOUS LIGAMENT
    //==========================================================================
  
    if(addInterspinous){
        // create a deformable mesh
        defInterspinous = new cGELMesh();
        defWorld->m_gelMeshes.push_front(defInterspinous);

	// position the mesh
        defInterspinous->setLocalPos(generalPos);

	// load the model
        bool fileload4;
	fileload4 = defInterspinous->loadFromFile(RESOURCE_PATH("../resources/models/interspinous/ISL2.stl"));
        if (!fileload4)
        {
            #if defined(_MSVC)
	    fileload4 = defInterspinous->loadFromFile("../../../bin/resources/models/interspinous/ISL2.stl");
            #endif
        }
        if (!fileload4)
        {
            printf("Error - Interspinous Ligament 3D Model failed to load correctly.\n");
            close();
            return (-1);
        }

        // resize model (from meters to centimeters)
        defInterspinous->scale(scale);

        // set some material color on the object
        cMaterial mat4;
        mat4.setWhite();
        mat4.setShininess(100);
        defInterspinous->setMaterial(mat4, true);

        // let's create a some environment mapping
        shared_ptr<cTexture2d> texture4(new cTexture2d());
        fileload4 = texture4->loadFromFile(RESOURCE_PATH("../resources/images/Ligament.jpg"));
        if (!fileload4)
        {
            #if defined(_MSVC)
            fileload4 = texture4->loadFromFile("../../../bin/resources/images/Ligament.jpg");
            #endif
        }
        if (!fileload4)
        {
            cout << "Error - Interspinous Ligament texture failed to load correctly." << endl;
            close();
            return (-1);
        }

        // enable environmental texturing
        texture4->setEnvironmentMode(GL_DECAL);
        texture4->setSphericalMappingEnabled(true);

        // assign and enable texturing
        defInterspinous->setTexture(texture4, true);
        defInterspinous->setUseTexture(true, true);

        // set object to be transparent
        defInterspinous->setTransparencyLevel(0.45, true, true);

        // build dynamic vertices
        defInterspinous->buildVertices();

        // set default properties for skeleton nodes
        cGELSkeletonNode::s_default_radius        = 0.02;  // [m]
        cGELSkeletonNode::s_default_kDampingPos   = 500.0;
        cGELSkeletonNode::s_default_kDampingRot   = 0.6;
        cGELSkeletonNode::s_default_mass          = 0.001; // [kg]
        cGELSkeletonNode::s_default_showFrame     = true;
        cGELSkeletonNode::s_default_color.setBlueCornflower();
        cGELSkeletonNode::s_default_useGravity    = false;
        cGELSkeletonNode::s_default_gravity.set(0.00, 0.00,-9.81);
        radius = cGELSkeletonNode::s_default_radius;

        // use internal skeleton as deformable model
        defInterspinous->m_useSkeletonModel = true;

        //------------Create Nodes------------------------------------------------//
        // get number of vertices
        int numVertices4 = defInterspinous->getNumVertices();
        int numNodes4 = int(numVertices4/factor);
	numNodesTot = numNodesTot + numNodes4;
        cout << "Interspinous Ligament number of nodes: " << endl;
        cout << numNodes4 << endl;

        // create nodes array
        cGELSkeletonNode* nodes4[numNodes4];

        // create an array of nodes
        for (int i=0; i<numNodes4; i++)
        {
            cGELSkeletonNode* newNode = new cGELSkeletonNode();
            nodes4[i] = newNode;
            defInterspinous->m_nodes.push_front(newNode);

            // get current deformable vertex
            cGELVertex* curVertex = &defInterspinous->m_gelVertices[i*factor];

            // get current vertex position
            cVector3d posVertex = curVertex->m_mesh->m_vertices->getLocalPos(curVertex->m_vertexIndex);
            double posx = posVertex(0);
            double posy = posVertex(1);
            double posz = posVertex(2);
            newNode->m_pos.set(posx, posy, posz);

        }


        //------------------------------------------------------------------------//

        // set corner nodes as fixed
        // 5% of nodes will be fixed
        int fixedNodes4 = int(numNodes4*0.05);
        for (int i=0; i<fixedNodes4-1; i++){

            // one node fixed every 20 (5%)
            nodes4[i*20]->m_fixed = true;

        }

        // set default physical properties for links
        cGELSkeletonLink::s_default_kSpringElongation = 100.0;  // [N/m]
        cGELSkeletonLink::s_default_kSpringFlexion    = 0.5;   // [Nm/RAD]
        cGELSkeletonLink::s_default_kSpringTorsion    = 0.1;   // [Nm/RAD]
        cGELSkeletonLink::s_default_color.setBlueCornflower();


        //------------Create Links between Nodes----------------------------------//
        for (int i=0; i<numNodes4-1; i++)
        {
            // cGELSkeletonLink* newLink = new cGELSkeletonLink(nodes[i], nodes[i+1]); 
            // defInterspinous->m_links.push_front(newLink);

            // get current deformable vertex
            cGELVertex* curVertex = &defInterspinous->m_gelVertices[i*factor];

            // get current vertex position
            cVector3d pos = curVertex->m_mesh->m_vertices->getLocalPos(curVertex->m_vertexIndex);

            // initialize constant
            double min_distance = 99999999999999999.0;
            cGELSkeletonNode* nearest_node = NULL;
            cGELSkeletonNode* almost_nearest_node = NULL;

            // search for the 2 nearest nodes
            list<cGELSkeletonNode*>::iterator itr;
            for(itr = defInterspinous->m_nodes.begin(); itr != defInterspinous->m_nodes.end(); ++itr)
            {
                cGELSkeletonNode* nextNode = *itr;
                double distance = cDistance(pos, nextNode->m_pos);
                if (distance < min_distance)
                {
                    min_distance = distance;
                    nearest_node = nextNode;
                    almost_nearest_node = nearest_node;
                }
            }

            // create a link with the two nearest nodes of the mesh
            cGELSkeletonLink* nearest_link = new cGELSkeletonLink(nodes4[i], nearest_node);
            cGELSkeletonLink* almost_nearest_link = new cGELSkeletonLink(nodes4[i], almost_nearest_node);
            defInterspinous->m_links.push_front(nearest_link);
            defInterspinous->m_links.push_front(almost_nearest_link);        
        }
        //------------------------------------------------------------------------//

        // connect skin (mesh) to skeleton (GEM). if __false__ then skin mesh is connected to nodes and links.
        defInterspinous->connectVerticesToSkeleton(false);

        // show/hide underlying dynamic skeleton model
        defInterspinous->m_showSkeletonModel = false;
    }

    //==========================================================================
    // 5 - INTRASPINOUS LIGAMENT
    //==========================================================================
  
    if(addSupraspinous){
        // create a deformable mesh
        defSupraspinous = new cGELMesh();
        defWorld->m_gelMeshes.push_front(defSupraspinous);

	// position the mesh
        defSupraspinous->setLocalPos(generalPos);

	// load the model
        bool fileload5;
	fileload5 = defSupraspinous->loadFromFile(RESOURCE_PATH("../resources/models/supraspinous/SSL23.stl"));
        if (!fileload5)
        {
            #if defined(_MSVC)
	    fileload5 = defSupraspinous->loadFromFile("../../../bin/resources/models/supraspinous/SSL23.stl");
            #endif
        }
        if (!fileload5)
        {
            printf("Error - Supraspinous Ligament 3D Model failed to load correctly.\n");
            close();
            return (-1);
        }

        // resize model (from meters to centimeters)
        defSupraspinous->scale(scale);

        // set some material color on the object
        cMaterial mat5;
        mat5.setWhite();
        mat5.setShininess(100);
        defSupraspinous->setMaterial(mat5, true);

        // let's create a some environment mapping
        shared_ptr<cTexture2d> texture5(new cTexture2d());
        fileload5 = texture5->loadFromFile(RESOURCE_PATH("../resources/images/Ligament.jpg"));
        if (!fileload5)
        {
            #if defined(_MSVC)
            fileload5 = texture5->loadFromFile("../../../bin/resources/images/Ligament.jpg");
            #endif
        }
        if (!fileload5)
        {
            cout << "Error - Supraspinous Ligament texture failed to load correctly." << endl;
            close();
            return (-1);
        }

        // enable environmental texturing
        texture5->setEnvironmentMode(GL_DECAL);
        texture5->setSphericalMappingEnabled(true);

        // assign and enable texturing
        defSupraspinous->setTexture(texture5, true);
        defSupraspinous->setUseTexture(true, true);

        // set object to be transparent
        defSupraspinous->setTransparencyLevel(0.45, true, true);

        // build dynamic vertices
        defSupraspinous->buildVertices();

        // set default properties for skeleton nodes
        cGELSkeletonNode::s_default_radius        = 0.02;  // [m]
        cGELSkeletonNode::s_default_kDampingPos   = 500.0;
        cGELSkeletonNode::s_default_kDampingRot   = 0.6;
        cGELSkeletonNode::s_default_mass          = 0.001; // [kg]
        cGELSkeletonNode::s_default_showFrame     = true;
        cGELSkeletonNode::s_default_color.setBlueCornflower();
        cGELSkeletonNode::s_default_useGravity    = false;
        cGELSkeletonNode::s_default_gravity.set(0.00, 0.00,-9.81);
        radius = cGELSkeletonNode::s_default_radius;

        // use internal skeleton as deformable model
        defSupraspinous->m_useSkeletonModel = true;

        //------------Create Nodes------------------------------------------------//
        // get number of vertices
        int numVertices5 = defSupraspinous->getNumVertices();
        int numNodes5 = int(numVertices5/factor);
	numNodesTot = numNodesTot + numNodes5;
        cout << "Supraspinous Ligament number of nodes: " << endl;
        cout << numNodes5 << endl;

        // create nodes array
        cGELSkeletonNode* nodes5[numNodes5];

        // create an array of nodes
        for (int i=0; i<numNodes5; i++)
        {
            cGELSkeletonNode* newNode = new cGELSkeletonNode();
            nodes5[i] = newNode;
            defSupraspinous->m_nodes.push_front(newNode);

            // get current deformable vertex
            cGELVertex* curVertex = &defSupraspinous->m_gelVertices[i*factor];

            // get current vertex position
            cVector3d posVertex = curVertex->m_mesh->m_vertices->getLocalPos(curVertex->m_vertexIndex);
            double posx = posVertex(0);
            double posy = posVertex(1);
            double posz = posVertex(2);
            newNode->m_pos.set(posx, posy, posz);

        }


        //------------------------------------------------------------------------//

        // set corner nodes as fixed
        // 5% of nodes will be fixed
        int fixedNodes5 = int(numNodes5*0.05);
        for (int i=0; i<fixedNodes5-1; i++){

            // one node fixed every 20 (5%)
            nodes5[i*20]->m_fixed = true;

        }

        // set default physical properties for links
        cGELSkeletonLink::s_default_kSpringElongation = 100.0;  // [N/m]
        cGELSkeletonLink::s_default_kSpringFlexion    = 0.5;   // [Nm/RAD]
        cGELSkeletonLink::s_default_kSpringTorsion    = 0.1;   // [Nm/RAD]
        cGELSkeletonLink::s_default_color.setBlueCornflower();


        //------------Create Links between Nodes----------------------------------//
        for (int i=0; i<numNodes5-1; i++)
        {
            // cGELSkeletonLink* newLink = new cGELSkeletonLink(nodes[i], nodes[i+1]); 
            // defSupraspinous->m_links.push_front(newLink);

            // get current deformable vertex
            cGELVertex* curVertex = &defSupraspinous->m_gelVertices[i*factor];

            // get current vertex position
            cVector3d pos = curVertex->m_mesh->m_vertices->getLocalPos(curVertex->m_vertexIndex);

            // initialize constant
            double min_distance = 99999999999999999.0;
            cGELSkeletonNode* nearest_node = NULL;
            cGELSkeletonNode* almost_nearest_node = NULL;

            // search for the 2 nearest nodes
            list<cGELSkeletonNode*>::iterator itr;
            for(itr = defSupraspinous->m_nodes.begin(); itr != defSupraspinous->m_nodes.end(); ++itr)
            {
                cGELSkeletonNode* nextNode = *itr;
                double distance = cDistance(pos, nextNode->m_pos);
                if (distance < min_distance)
                {
                    min_distance = distance;
                    nearest_node = nextNode;
                    almost_nearest_node = nearest_node;
                }
            }

            // create a link with the two nearest nodes of the mesh
            cGELSkeletonLink* nearest_link = new cGELSkeletonLink(nodes5[i], nearest_node);
            cGELSkeletonLink* almost_nearest_link = new cGELSkeletonLink(nodes5[i], almost_nearest_node);
            defSupraspinous->m_links.push_front(nearest_link);
            defSupraspinous->m_links.push_front(almost_nearest_link);        
        }
        //------------------------------------------------------------------------//

        // connect skin (mesh) to skeleton (GEM). if __false__ then skin mesh is connected to nodes and links.
        defSupraspinous->connectVerticesToSkeleton(false);

        // show/hide underlying dynamic skeleton model
        defSupraspinous->m_showSkeletonModel = false;
    }

    //==========================================================================
    // 6 - FLAVUM LIGAMENT
    //==========================================================================

    if(addFlavum){
        // create a deformable mesh
        defFlavum = new cGELMesh();
        defWorld->m_gelMeshes.push_front(defFlavum);

	// position the mesh
        defFlavum->setLocalPos(generalPos);

	// load the model
        bool fileload6;
	fileload6 = defFlavum->loadFromFile(RESOURCE_PATH("../resources/models/flavum/flavo.stl"));
        if (!fileload6)
        {
            #if defined(_MSVC)
	    fileload6 = defFlavum->loadFromFile("../../../bin/resources/models/flavum/flavo.stl");
            #endif
        }
        if (!fileload6)
        {
            printf("Error - Flavum ligament 3D Model failed to load correctly.\n");
            close();
            return (-1);
        }

        // resize model (from meters to centimeters)
        defFlavum->scale(scale);

        // set some material color on the object
        cMaterial mat6;
        mat6.setWhite();
        mat6.setShininess(100);
        defFlavum->setMaterial(mat6, true);

        // let's create a some environment mapping
        shared_ptr<cTexture2d> texture6(new cTexture2d());
        fileload6 = texture6->loadFromFile(RESOURCE_PATH("../resources/images/Ligament.jpg"));
        if (!fileload6)
        {
            #if defined(_MSVC)
            fileload6 = texture6->loadFromFile("../../../bin/resources/images/Ligament.jpg");
            #endif
        }
        if (!fileload6)
        {
            cout << "Error - Ligament texture failed to load correctly." << endl;
            close();
            return (-1);
        }

        // enable environmental texturing
        texture6->setEnvironmentMode(GL_DECAL);
        texture6->setSphericalMappingEnabled(true);

        // assign and enable texturing
        defFlavum->setTexture(texture6, true);
        defFlavum->setUseTexture(true, true);

        // set object to be transparent
        defFlavum->setTransparencyLevel(0.75, true, true);

        // build dynamic vertices
        defFlavum->buildVertices();

        // set default properties for skeleton nodes
        cGELSkeletonNode::s_default_radius        = 0.02;  // [m]
        cGELSkeletonNode::s_default_kDampingPos   = 500.0;
        cGELSkeletonNode::s_default_kDampingRot   = 0.6;
        cGELSkeletonNode::s_default_mass          = 0.001; // [kg]
        cGELSkeletonNode::s_default_showFrame     = true;
        cGELSkeletonNode::s_default_color.setRed();
        cGELSkeletonNode::s_default_useGravity    = false;
        cGELSkeletonNode::s_default_gravity.set(0.00, 0.00,-9.81);
        radius = cGELSkeletonNode::s_default_radius;

        // use internal skeleton as deformable model
        defFlavum->m_useSkeletonModel = true;

        //------------Create Nodes------------------------------------------------//
        // get number of vertices
        int numVertices6 = defFlavum->getNumVertices();
        int numNodes6 = int(numVertices6/factor);
	numNodesTot = numNodesTot + numNodes6;
        cout << "Flavum ligament number of nodes: " << endl;
        cout << numNodes6 << endl;
	cout << "------------------------------" << endl;
	cout << "Total number of nodes: " << endl;
	cout << numNodesTot << endl;

        // create nodes array
        cGELSkeletonNode* nodes6[numNodes6];

        // create an array of nodes
        for (int i=0; i<numNodes6; i++)
        {
            cGELSkeletonNode* newNode = new cGELSkeletonNode();
            nodes6[i] = newNode;
            defFlavum->m_nodes.push_front(newNode);

            // get current deformable vertex
            cGELVertex* curVertex = &defFlavum->m_gelVertices[i*factor];

            // get current vertex position
            cVector3d posVertex = curVertex->m_mesh->m_vertices->getLocalPos(curVertex->m_vertexIndex);
            double posx = posVertex(0);
            double posy = posVertex(1);
            double posz = posVertex(2);
            newNode->m_pos.set(posx, posy, posz);

        }


        //------------------------------------------------------------------------//

        // set corner nodes as fixed
        // 5% of nodes will be fixed
        int fixedNodes6 = int(numNodes6*0.05);
        for (int i=0; i<fixedNodes6-1; i++){

            // one node fixed every 20 (10%)
            nodes6[i*20]->m_fixed = true;

        }

        // set default physical properties for links
        cGELSkeletonLink::s_default_kSpringElongation = 50.0;  // [N/m]
        cGELSkeletonLink::s_default_kSpringFlexion    = 0.5;   // [Nm/RAD]
        cGELSkeletonLink::s_default_kSpringTorsion    = 0.1;   // [Nm/RAD]
        cGELSkeletonLink::s_default_color.setBlueCornflower();


        //------------Create Links between Nodes----------------------------------//
        for (int i=0; i<numNodes6-1; i++)
        {
            // cGELSkeletonLink* newLink = new cGELSkeletonLink(nodes[i], nodes[i+1]); 
            // defMuscle->m_links.push_front(newLink);

            // get current deformable vertex
            cGELVertex* curVertex = &defFlavum->m_gelVertices[i*factor];

            // get current vertex position
            cVector3d pos = curVertex->m_mesh->m_vertices->getLocalPos(curVertex->m_vertexIndex);

            // initialize constant
            double min_distance = 99999999999999999.0;
            cGELSkeletonNode* nearest_node = NULL;
            cGELSkeletonNode* almost_nearest_node = NULL;

            // search for the 2 nearest nodes
            list<cGELSkeletonNode*>::iterator itr;
            for(itr = defFlavum->m_nodes.begin(); itr != defFlavum->m_nodes.end(); ++itr)
            {
                cGELSkeletonNode* nextNode = *itr;
                double distance = cDistance(pos, nextNode->m_pos);
                if (distance < min_distance)
                {
                    min_distance = distance;
                    nearest_node = nextNode;
                    almost_nearest_node = nearest_node;
                }
            }

            // create a link with the two nearest nodes of the mesh
            cGELSkeletonLink* nearest_link = new cGELSkeletonLink(nodes6[i], nearest_node);
            cGELSkeletonLink* almost_nearest_link = new cGELSkeletonLink(nodes6[i], almost_nearest_node);
            defFlavum->m_links.push_front(nearest_link);
            defFlavum->m_links.push_front(almost_nearest_link);        
        }
        //------------------------------------------------------------------------//

        // connect skin (mesh) to skeleton (GEM). if __false__ then skin mesh is connected to nodes and links.
        defFlavum->connectVerticesToSkeleton(false);

        // show/hide underlying dynamic skeleton model
        defFlavum->m_showSkeletonModel = false;
    }
    
    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    cFont *font = NEW_CFONTCALIBRI20();

    // create a label to display the haptic rate of the simulation
    labelHapticRate = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticRate);
    labelHapticRate->m_fontColor.setBlack();
    
    // create a label to display the position of haptic device
    labelHapticDevicePosition = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticDevicePosition);
    labelHapticDevicePosition->m_fontColor.setBlack();


    // create a background
    cBackground* background = new cBackground();
    camera->m_backLayer->addChild(background);

    // set background properties
    //background->setCornerColors(cColorf(1.00f, 1.00f, 1.00f),
                            //    cColorf(0.95f, 0.95f, 0.95f),
                            //    cColorf(0.85f, 0.85f, 0.85f),
                            //    cColorf(0.80f, 0.80f, 0.80f));
    

    //-----------------------------------------------------------------------
    // START SIMULATION
    //-----------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    cThread* hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);

    // start the main graphics rendering loop
    glutTimerFunc(50, graphicsTimer, 0);
    glutMainLoop();

    // exit
    return (0);
}

//---------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

outputFile.close();
}

//---------------------------------------------------------------------------

void graphicsTimer(int data)
{
    if (simulationRunning)
    {
        glutPostRedisplay();
    }

    glutTimerFunc(50, graphicsTimer, 0);
}

//---------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update haptic rate label
    labelHapticRate->setText (cStr(frequencyCounter.getFrequency(), 0) + " Hz");

    // update position of label
    labelHapticRate->setLocalPos((int)(0.5 * (windowW - labelHapticRate->getWidth())), 15);

    // display new position data
    labelHapticDevicePosition->setText(hapticDevicePosition.str(3));

    // update position of label
    labelHapticDevicePosition->setLocalPos(20, windowH - 60, 0);
    
    /////////////////////////////////////////////////////////////////////
    // UPDATE DEFORMABLE MODELS
    /////////////////////////////////////////////////////////////////////

    // update skins deformable objects
    defWorld->updateSkins(true);


    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, mirroredDisplay);

    // render world
    camera->renderView(windowW, windowH);

    // swap buffers
    glutSwapBuffers();

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}

//---------------------------------------------------------------------------

void updateHaptics(void)
{
    // initialize frequency counter
    frequencyCounter.reset();

    // initialize precision clock
    cPrecisionClock clock;
    clock.reset();

    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    // main haptic simulation loop
    while(simulationRunning)
    {
        // stop clock
        double time = cMin(0.001, clock.stop());
        // restart clock
        clock.start(true);

        // initial position of the haptic device
        cVector3d pos;

	/* Receive string from Arduino */
	n = read(fd, buf, 1);
	pos(0)=((int)buf[0]);

	//printf("Pos:%i, ", (int)buf[0]);
	//cout << pos(2) <<endl;

       // hapticDevice->getPosition(pos);
        pos.mul(-workspaceScaleFactor);
	pos(0) = pos(0) + 0.08;  //8 cm to the center of the disk along the x axis
	pos(1) = 0.0;//0.05;	  //clear position in the y axis
	pos(2) = 0.0;//0.01;    //clear position in the z axis
        device->setLocalPos(pos);

        // clear all external forces
        defWorld->clearExternalForces();

        // compute reaction forces
        cVector3d force(0.0, 0.0, 0.0);
        list<cGELMesh*>::iterator i1;
        list<cGELSkeletonNode*>::iterator i2;	

        // for all the different meshes
        for(i1 = defWorld->m_gelMeshes.begin(); i1 != defWorld->m_gelMeshes.end(); ++i1)
        {
            cGELMesh *nextMesh = *i1;
            // for every node of each mesh
            for(i2 = nextMesh->m_nodes.begin(); i2 != nextMesh->m_nodes.end(); ++i2)
            {
                cGELSkeletonNode* nextItem = *i2;

                //Node local position + mesh position to get global position
                cVector3d nodePos = nextItem->m_pos + generalPos;
                cVector3d f = computeForce(pos, deviceRadius, nodePos, radius, stiffness);
                cVector3d tmpfrc = -1.0 * f;
                nextItem->setExternalForce(tmpfrc);
		force(1) = 0.0; //clear force in the y axis
		force(2) = 0.0; //clear force in the z axis
                force.add(f);
            }
        }
	
        // integrate dynamics
        defWorld->updateDynamics(time);
	
        // scale force
        force.mul(deviceForceScale / workspaceScaleFactor);

	int pwm = force(0)*100/400;
	//printf("pwm:%i, ", pwm);

	// to save force data in a text file	
	if (saveForces == true){
	  outputFile << pwm;
	  outputFile << " ";
	  outputFile << pos(0);
	  outputFile << endl;
	}

	/* Send force byte to Arduino*/
	int size = write(fd, &pwm, 1); 
	
        //----------------------------------------------------------------------
        
        // update frequency counter
        frequencyCounter.signal(1);
        
        // update global variable for graphic display update
        hapticDevicePosition = pos;
    }

    // exit haptics thread
    simulationFinished = true;
}

//---------------------------------------------------------------------------

cVector3d computeForce(const cVector3d& a_cursor,
                       double a_cursorRadius,
                       const cVector3d& a_spherePos,
                       double a_radius,
                       double a_stiffness)
{

    // In the following we compute the reaction forces between the tool and the
    // sphere.
    cVector3d force;
    force.zero();

    cVector3d vSphereCursor = a_cursor - a_spherePos;
    double dist = vSphereCursor.length();

    if (dist < 0.0000001)
    {
        return (force);
    }

    if (dist > (a_cursorRadius + a_radius))
    {
        return (force);
    }

    // compute penetration distance between tool and surface of sphere
    double penetrationDistance = (a_cursorRadius + a_radius) - vSphereCursor.length();
    cVector3d forceDirection = cNormalize(vSphereCursor);
    force = cMul( penetrationDistance * a_stiffness, forceDirection);

    return (force);
}
