/*
  CSCI 420 Computer Graphics
  Assignment 2 Roller Coasters
  Zhouqian Lu
 */

#include <stdio.h>
#include <stdlib.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#include "pic.h"
#include <math.h>
#include <cstring>
#include <vector>

int g_iMenuId;

int g_vMousePos[2] = {0, 0};
int g_iLeftMouseButton = 0; /* 1 if pressed, 0 if not */
int g_iMiddleMouseButton = 0;
int g_iRightMouseButton = 0;
int cameraPoint = 0;

typedef enum
{
    ROTATE,
    TRANSLATE,
    SCALE
} CONTROLSTATE;

CONTROLSTATE g_ControlState = ROTATE;
/* state of the world */
float g_vLandRotate[3] = {0.0, 0.0, 0.0};
float g_vLandTranslate[3] = {0.0, 0.0, 0.0};
float g_vLandScale[3] = {1.0, 1.0, 1.0};

// Camera coordinates
float camera_coordinates[3] = {0.0, 0.0, 0.0};
float camera_forward[3] = {0.0, 0.0, 0.0};
float camera_up[3] = {0.0, 0.0, 0.0};
float camera_right[3] = {0.0, 1.0, 0.0};

/* represents one control point along the spline */
struct point
{
    double x;
    double y;
    double z;
};

/* spline struct which contains how many control points, and an array of control points */
struct spline
{
    int numControlPoints;
    struct point *points;
};

/* the spline array */
struct spline *g_Splines;

/* total number of splines */
int g_iNumOfSplines;

std::vector<point> pointList;

/* stores the image */
Pic *g_pSkyPic;
Pic *g_pGroundPic;

GLuint skyTex;
GLuint groundTex;

/* Basis matrix for Catmull-Rom */
float s = 0.5;
float basis_matrix[4][4] = {{-s, 2 - s, s - 2, s}, {2 * s, s - 3, 3 - 2 * s, -s}, {-s, 0, s, 0}, {0, 1, 0, 0}};

int loadSplines(char *argv)
{
    char *cName = (char *)malloc(128 * sizeof(char));
    FILE *fileList;
    FILE *fileSpline;
    int iType, i = 0, j, iLength;

    /* load the track file */
    fileList = fopen(argv, "r");
    if (fileList == NULL)
    {
        printf("can't open file\n");
        exit(1);
    }

    /* stores the number of splines in a global variable */
    fscanf(fileList, "%d", &g_iNumOfSplines);

    g_Splines = (struct spline *)malloc(g_iNumOfSplines * sizeof(struct spline));

    /* reads through the spline files */
    for (j = 0; j < g_iNumOfSplines; j++)
    {
        i = 0;
        fscanf(fileList, "%s", cName);
        fileSpline = fopen(cName, "r");

        if (fileSpline == NULL)
        {
            printf("can't open file\n");
            exit(1);
        }

        /* gets length for spline file */
        fscanf(fileSpline, "%d %d", &iLength, &iType);

        /* allocate memory for all the points */
        g_Splines[j].points = (struct point *)malloc(iLength * sizeof(struct point));
        g_Splines[j].numControlPoints = iLength;

        /* saves the data to the struct */
        while (fscanf(fileSpline, "%lf %lf %lf",
                      &g_Splines[j].points[i].x,
                      &g_Splines[j].points[i].y,
                      &g_Splines[j].points[i].z) != EOF)
        {
            i++;
        }
    }

    free(cName);

    return 0;
}

/* initilize texture for sky and ground */
void initTexture(GLuint *texture, Pic *image)
{
    int width = image->nx;
    int height = image->ny;
    unsigned char *data = new unsigned char[4 * width * height];
    glGenTextures(1, texture);
    memset(data, 0, 4 * width * height);
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            data[4 * (i * width + j) + 0] = 0;
            data[4 * (i * width + j) + 1] = 0;
            data[4 * (i * width + j) + 2] = 0;
            data[4 * (i * width + j) + 3] = 255;

            int numChannels = image->bpp;
            for (int k = 0; k < numChannels; k++)
            {
                data[4 * (i * width + j) + k] = PIC_PIXEL(image, j, i, k);
            }
        }
    }

    /* makes texture skytex the current active texture */
    glBindTexture(GL_TEXTURE_2D, *texture);

    /* specifies texture parameters, repeats pattern in s and t */
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    /* users linear filter both for magnificaiton and minification */
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    /* load image data into the active texture */
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);

    delete[] data;
}

/* performs the 4*4 matrix matrix multiplication */
void matrix_44_multiply(float A[4][4], float B[4][4], float result[4][4])
{
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            float sum = 0.0f;
            for (int k = 0; k < 4; k++)
            {
                sum += A[i][k] * B[k][j];
            }
            result[i][j] = sum;
        }
    }
}

/* performs the 4*4 and 4*3 matrix multiplication */
void matrix_43_multiply(float A[4][4], float B[4][3], float result[4][3])
{
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            float sum = 0.0f;
            for (int k = 0; k < 4; k++)
            {
                sum += A[i][k] * B[k][j];
            }
            result[i][j] = sum;
        }
    }
}

void vector_matrix_multiply(float A[4], float B[4][3], float result[3])
{
    for (int i = 0; i < 3; i++)
    {
        float sum = 0.0f;
        for (int k = 0; k < 4; k++)
        {
            sum += A[k] * B[k][i];
        }
        result[i] = sum;
    }
}

void normalize(float p[3])
{
    float len = sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
    p[0] /= len;
    p[1] /= len;
    p[2] /= len;
}

void cross_product(float a[3], float b[3], float c[3])
{
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}

// Draws splines for the roller coaster
void drawSplines()
{
    // Brute force approach
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i < g_iNumOfSplines; i++)
    {
        for (int j = 1; j < g_Splines[i].numControlPoints - 2; j++)
        {
            float u = 0.0f;
            float p[3] = {0, 0, 0};
            float control_matrix[4][3] = {{g_Splines[i].points[j - 1].x, g_Splines[i].points[j - 1].y, g_Splines[i].points[j - 1].z},
                                          {g_Splines[i].points[j].x, g_Splines[i].points[j].y, g_Splines[i].points[j].z},
                                          {g_Splines[i].points[j + 1].x, g_Splines[i].points[j + 1].y, g_Splines[i].points[j + 1].z},
                                          {g_Splines[i].points[j + 2].x, g_Splines[i].points[j + 2].y, g_Splines[i].points[j + 2].z}};

            while (u <= 1.0)
            {
                float uVector[4] = {pow(u, 3), pow(u, 2), u, 1};
                float coefficient[4][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

                matrix_43_multiply(basis_matrix, control_matrix, coefficient);
                // Calculate the x y coordinates
                vector_matrix_multiply(uVector, coefficient, p);
                // glColor3f(1, 0, 0);
                normalize(p);
                point thisPoint;
                thisPoint.x = p[0];
                thisPoint.y = p[1];
                thisPoint.z = p[2];
                glVertex3d(p[0], p[1], p[2]);
                pointList.push_back(thisPoint);
                u += 0.001f;
            }
        }
    }
    glEnd();
}

void drawGround()
{
    glBindTexture(GL_TEXTURE_2D, groundTex);
    glEnable(GL_TEXTURE_2D);

    // Draws texture quads
    glBegin(GL_QUADS);
    glTexCoord2f(0.0, 0.0);
    glVertex3f(-1.0, -1.0, -1.0);
    glTexCoord2f(0.0, 1.0);
    glVertex3f(-1.0, 1.0, -1.0);
    glTexCoord2f(1.0, 1.0);
    glVertex3f(1.0, 1.0, -1.0);
    glTexCoord2f(1.0, 0.0);
    glVertex3f(1.0, -1.0, -1.0);
    glEnd();

    // turns off texture mapping
    glDisable(GL_TEXTURE_2D);
}

void drawSky()
{
    glBindTexture(GL_TEXTURE_2D, skyTex);
    glEnable(GL_TEXTURE_2D);

    // Draws texture quads
    glBegin(GL_QUADS);
    glTexCoord2f(0.0, 0.0);
    glVertex3f(-1.0, -1.0, 1.0);
    glTexCoord2f(0.0, 1.0);
    glVertex3f(-1.0, 1.0, 1.0);
    glTexCoord2f(1.0, 1.0);
    glVertex3f(1.0, 1.0, 1.0);
    glTexCoord2f(1.0, 0.0);
    glVertex3f(1.0, -1.0, 1.0);

    glTexCoord2f(0.0, 0.0);
    glVertex3f(-1.0, 1.0, 1.0);
    glTexCoord2f(0.0, 1.0);
    glVertex3f(1.0, 1.0, 1.0);
    glTexCoord2f(1.0, 1.0);
    glVertex3f(1.0, 1.0, -1.0);
    glTexCoord2f(1.0, 0.0);
    glVertex3f(-1.0, 1.0, -1.0);

    glTexCoord2f(0.0, 0.0);
    glVertex3f(-1.0, -1.0, 1.0);
    glTexCoord2f(0.0, 1.0);
    glVertex3f(1.0, -1.0, 1.0);
    glTexCoord2f(1.0, 1.0);
    glVertex3f(1.0, -1.0, -1.0);
    glTexCoord2f(1.0, 0.0);
    glVertex3f(-1.0, -1.0, -1.0);

    glTexCoord2f(0.0, 0.0);
    glVertex3f(-1.0, -1.0, 1.0);
    glTexCoord2f(0.0, 1.0);
    glVertex3f(-1.0, -1.0, -1.0);
    glTexCoord2f(1.0, 1.0);
    glVertex3f(-1.0, 1.0, -1.0);
    glTexCoord2f(1.0, 0.0);
    glVertex3f(-1.0, 1.0, 1.0);

    glTexCoord2f(0.0, 0.0);
    glVertex3f(1.0, -1.0, 1.0);
    glTexCoord2f(0.0, 1.0);
    glVertex3f(1.0, -1.0, -1.0);
    glTexCoord2f(1.0, 1.0);
    glVertex3f(1.0, 1.0, -1.0);
    glTexCoord2f(1.0, 0.0);
    glVertex3f(1.0, 1.0, 1.0);

    glEnd();

    // turns off texture mapping
    glDisable(GL_TEXTURE_2D);
}

void ride()
{
    float x = pointList[cameraPoint].x;
    float y = pointList[cameraPoint].y;
    float z = pointList[cameraPoint].z;
    cameraPoint += 1;
    camera_forward[0] = pointList[cameraPoint].x - x;
    camera_forward[1] = pointList[cameraPoint].y - y;
    camera_forward[2] = pointList[cameraPoint].z - z;
    normalize(camera_forward);
    cross_product(camera_forward, camera_right, camera_up);
    normalize(camera_up);
    cross_product(camera_up, camera_forward, camera_right);
    normalize(camera_right);
    if (cameraPoint >= pointList.size())
    {
        cameraPoint = 0;
    }
}

void myinit()
{
    // sets up gl view here
    glClearColor(0.0, 0.0, 0.0, 0.0);

    // enable depth buffering
    glEnable(GL_DEPTH_TEST);

    // color interpolation
    glShadeModel(GL_SMOOTH);

    g_pSkyPic = jpeg_read("sky_texture.jpg", NULL);
    g_pGroundPic = jpeg_read("ground_texture.jpg", NULL);

    if (!g_pSkyPic || !g_pGroundPic)
    {
        printf("error reading texture files.\n");
        exit(1);
    }

    initTexture(&skyTex, g_pSkyPic);
    initTexture(&groundTex, g_pGroundPic);
    // ride();
}

void display()
{
    /* clear buffers */
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);

    // no modulation of texture color with lighting; use texture color directly
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

    /* object coordinate is the same as the world coordinate */
    glLoadIdentity();
    // ride();
    //   sets the camera position
    // gluLookAt(camera_coordinates[0], camera_coordinates[1], camera_coordinates[2], camera_coordinates[0] + camera_forward[0], camera_coordinates[1] + camera_forward[1], camera_coordinates[2] + camera_forward[2], camera_up[0], camera_up[1], camera_up[2]);

    gluLookAt(0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);

    // sets translation, rotation, and scale
    glTranslatef(g_vLandTranslate[0], g_vLandTranslate[1], g_vLandTranslate[2]);
    glRotatef(g_vLandRotate[0], 1, 0, 0);
    glRotatef(g_vLandRotate[1], 0, 1, 0);
    glRotatef(g_vLandRotate[2], 0, 0, 1);
    glScalef(g_vLandScale[0], g_vLandScale[1], g_vLandScale[2]);

    drawGround();
    drawSplines();

    drawSky();

    glutSwapBuffers();
}

void doIdle()
{
    /* do some stuff... */

    /* make the screen update */
    glutPostRedisplay();
}

/* converts mouse drags into information about
rotation/translation/scaling */
void mousedrag(int x, int y)
{
    int vMouseDelta[2] = {x - g_vMousePos[0], y - g_vMousePos[1]};

    switch (g_ControlState)
    {
    case TRANSLATE:
        if (g_iLeftMouseButton)
        {
            g_vLandTranslate[0] += vMouseDelta[0] * 0.01;
            g_vLandTranslate[1] -= vMouseDelta[1] * 0.01;
        }
        if (g_iMiddleMouseButton)
        {
            g_vLandTranslate[2] += vMouseDelta[1] * 0.01;
        }
        break;
    case ROTATE:
        if (g_iLeftMouseButton)
        {
            g_vLandRotate[0] += vMouseDelta[1];
            g_vLandRotate[1] += vMouseDelta[0];
        }
        if (g_iMiddleMouseButton)
        {
            g_vLandRotate[2] += vMouseDelta[1];
        }
        break;
    case SCALE:
        if (g_iLeftMouseButton)
        {
            g_vLandScale[0] *= 1.0 + vMouseDelta[0] * 0.01;
            g_vLandScale[1] *= 1.0 - vMouseDelta[1] * 0.01;
        }
        if (g_iMiddleMouseButton)
        {
            g_vLandScale[2] *= 1.0 - vMouseDelta[1] * 0.01;
        }
        break;
    }
    g_vMousePos[0] = x;
    g_vMousePos[1] = y;
}

void mouseidle(int x, int y)
{
    g_vMousePos[0] = x;
    g_vMousePos[1] = y;
}

void mousebutton(int button, int state, int x, int y)
{

    switch (button)
    {
    case GLUT_LEFT_BUTTON:
        g_iLeftMouseButton = (state == GLUT_DOWN);
        break;
    case GLUT_MIDDLE_BUTTON:
        g_iMiddleMouseButton = (state == GLUT_DOWN);
        break;
    case GLUT_RIGHT_BUTTON:
        g_iRightMouseButton = (state == GLUT_DOWN);
        break;
    }

    switch (glutGetModifiers())
    {
    case GLUT_ACTIVE_CTRL:
        g_ControlState = TRANSLATE;
        break;
    case GLUT_ACTIVE_SHIFT:
        g_ControlState = SCALE;
        break;
    default:
        g_ControlState = ROTATE;
        break;
    }

    g_vMousePos[0] = x;
    g_vMousePos[1] = y;
}

void menufunc(int value)
{
    switch (value)
    {
    case 0:
        exit(0);
        break;
    }
}

void reshape(int w, int h)
{
    GLfloat aspect = (GLfloat)w / (GLfloat)h;
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45, aspect, 0.001, 1000.0);
    glMatrixMode(GL_MODELVIEW);
}

void keyboard(unsigned char key, int x, int y)
{
    exit(0);
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        printf("usage: %s <trackfile>\n", argv[0]);
        exit(0);
    }

    // Initializes glut
    glutInit(&argc, argv);

    // request double buffer
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGBA);

    // set window size
    glutInitWindowSize(640, 480);

    // set window position
    glutInitWindowPosition(0, 0);

    // creates a window
    glutCreateWindow("rollercoaster!");

    loadSplines(argv[1]);

    myinit();

    glutDisplayFunc(display);

    /* allow the user to quit using the right mouse button menu */
    g_iMenuId = glutCreateMenu(menufunc);
    glutSetMenu(g_iMenuId);
    glutAddMenuEntry("Quit", 0);

    glutIdleFunc(doIdle);
    glutAttachMenu(GLUT_RIGHT_BUTTON);
    /* callback for mouse drags */
    glutMotionFunc(mousedrag);
    /* callback for idle mouse movement */
    glutPassiveMotionFunc(mouseidle);
    /* callback for mouse button changes */
    glutMouseFunc(mousebutton);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);

    glutMainLoop();
    return 0;
}
