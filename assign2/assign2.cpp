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
int cameraPoint = 1;

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
double cameraPos[3] = {0.0, 0.0, 0.0};
double tangent[3] = {0.0, 0.0, 0.0};
double N[3] = {0.0, 0.0, 0.0};
double B[3] = {0.0, 0.0, 0.0};

// track coordinates
double _tangent[3] = {0.0, 0.0, 0.0};
double _N[3] = {0.0, 0.0, 0.0};
double _B[3] = {0.0, 0.0, 0.0};

double alpha = 0.01;

double arbitrary_vector[3] = {0.0, 0.0, 1.0};

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
std::vector<point> trackList;
std::vector<point> trackList2;
std::vector<point> supportList;
std::vector<point> supportList2;

/* stores the image */
Pic *g_pSkyPic;
Pic *g_pGroundPic;
Pic *g_pTrackPic;
Pic *g_pTrack2Pic;
Pic *g_pSupportPic;

GLuint skyTex;
GLuint groundTex;
GLuint trackTex;
GLuint supportTex;
GLuint track2Tex;

/* Basis matrix for Catmull-Rom */
double s = 0.5;
double basis_matrix[4][4] = {{-s, 2 - s, s - 2, s}, {2 * s, s - 3, 3 - 2 * s, -s}, {-s, 0, s, 0}, {0, 1, 0, 0}};

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
void matrix_44_multiply(double A[4][4], double B[4][4], double result[4][4])
{
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      double sum = 0.0;
      for (int k = 0; k < 4; k++)
      {
        sum += A[i][k] * B[k][j];
      }
      result[i][j] = sum;
    }
  }
}

/* performs the 4*4 and 4*3 matrix multiplication */
void matrix_43_multiply(double A[4][4], double B[4][3], double result[4][3])
{
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      double sum = 0.0;
      for (int k = 0; k < 4; k++)
      {
        sum += A[i][k] * B[k][j];
      }
      result[i][j] = sum;
    }
  }
}

void vector_matrix_multiply(double A[4], double B[4][3], double result[3])
{
  for (int i = 0; i < 3; i++)
  {
    double sum = 0.0;
    for (int k = 0; k < 4; k++)
    {
      sum += A[k] * B[k][i];
    }
    result[i] = sum;
  }
}

void normalize(double p[3])
{
  double len = sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
  p[0] /= len;
  p[1] /= len;
  p[2] /= len;
}

void cross_product(double a[3], double b[3], double c[3])
{
  c[0] = a[1] * b[2] - a[2] * b[1];
  c[1] = a[2] * b[0] - a[0] * b[2];
  c[2] = a[0] * b[1] - a[1] * b[0];
}

// Draws splines for the roller coaster
void createSplines()
{
  for (int i = 0; i < g_iNumOfSplines; i++)
  {
    for (int j = 1; j < g_Splines[i].numControlPoints - 2; j++)
    {
      double u = 0.0;
      double p[3] = {0, 0, 0};
      double control_matrix[4][3] = {{g_Splines[i].points[j - 1].x, g_Splines[i].points[j - 1].y, g_Splines[i].points[j - 1].z},
                                     {g_Splines[i].points[j].x, g_Splines[i].points[j].y, g_Splines[i].points[j].z},
                                     {g_Splines[i].points[j + 1].x, g_Splines[i].points[j + 1].y, g_Splines[i].points[j + 1].z},
                                     {g_Splines[i].points[j + 2].x, g_Splines[i].points[j + 2].y, g_Splines[i].points[j + 2].z}};

      while (u <= 1.0)
      {
        double uVector[4] = {pow(u, 3), pow(u, 2), u, 1};
        double coefficient[4][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

        matrix_43_multiply(basis_matrix, control_matrix, coefficient);
        // Calculate the x y coordinates
        vector_matrix_multiply(uVector, coefficient, p);
        // glColor3f(1, 0, 0);
        normalize(p);
        point thisPoint;
        thisPoint.x = p[0];
        thisPoint.y = p[1];
        thisPoint.z = p[2];
        pointList.push_back(thisPoint);
        u += 0.01;
      }
    }
  }
}

void drawSplines()
{
  glBegin(GL_LINES);
  glLineWidth(5.0f);
  for (int i = 0; i < pointList.size() - 2; i++)
  {
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(pointList[i].x, pointList[i].y, pointList[i].z);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(pointList[i + 1].x, pointList[i + 1].y, pointList[i + 1].z);
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
  glVertex3f(-5.0, -5.0, 5.0);
  glTexCoord2f(0.0, 1.0);
  glVertex3f(-5.0, 5.0, 5.0);
  glTexCoord2f(1.0, 1.0);
  glVertex3f(5.0, 5.0, 5.0);
  glTexCoord2f(1.0, 0.0);
  glVertex3f(5.0, -5.0, 5.0);
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
  glVertex3f(-5.0, -5.0, -5.0);
  glTexCoord2f(0.0, 1.0);
  glVertex3f(-5.0, 5.0, -5.0);
  glTexCoord2f(1.0, 1.0);
  glVertex3f(5.0, 5.0, -5.0);
  glTexCoord2f(1.0, 0.0);
  glVertex3f(5.0, -5.0, -5.0);

  glTexCoord2f(0.0, 0.0);
  glVertex3f(-5.0, -5.0, 5.0);
  glTexCoord2f(0.0, 1.0);
  glVertex3f(5.0, -5.0, 5.0);
  glTexCoord2f(1.0, 1.0);
  glVertex3f(5.0, -5.0, -5.0);
  glTexCoord2f(1.0, 0.0);
  glVertex3f(-5.0, -5.0, -5.0);

  glTexCoord2f(0.0, 0.0);
  glVertex3f(-5.0, 5.0, 5.0);
  glTexCoord2f(0.0, 1.0);
  glVertex3f(-5.0, 5.0, -5.0);
  glTexCoord2f(1.0, 1.0);
  glVertex3f(5.0, 5.0, -5.0);
  glTexCoord2f(1.0, 0.0);
  glVertex3f(5.0, 5.0, 5.0);

  glTexCoord2f(0.0, 0.0);
  glVertex3f(-5.0, -5.0, 5.0);
  glTexCoord2f(0.0, 1.0);
  glVertex3f(-5.0, -5.0, -5.0);
  glTexCoord2f(1.0, 1.0);
  glVertex3f(-5.0, 5.0, -5.0);
  glTexCoord2f(1.0, 0.0);
  glVertex3f(-5.0, 5.0, 5.0);

  glTexCoord2f(0.0, 0.0);
  glVertex3f(5.0, -5.0, 5.0);
  glTexCoord2f(0.0, 1.0);
  glVertex3f(5.0, -5.0, -5.0);
  glTexCoord2f(1.0, 1.0);
  glVertex3f(5.0, 5.0, -5.0);
  glTexCoord2f(1.0, 0.0);
  glVertex3f(5.0, 5.0, 5.0);

  glEnd();

  // turns off texture mapping
  glDisable(GL_TEXTURE_2D);
}

/* Creates the track of the roller coaster using tangent, normal and binormal vectors */
void createTrack()
{
  for (int i = 1; i < pointList.size() - 2; i++)
  {
    _tangent[0] = pointList[i + 1].x - pointList[i - 1].x;
    _tangent[1] = pointList[i + 1].y - pointList[i - 1].y;
    _tangent[2] = pointList[i + 1].z - pointList[i - 1].z;

    normalize(_tangent);

    // Initialization
    if (i == 1)
    {
      cross_product(_tangent, arbitrary_vector, _N);
      normalize(_N);
      cross_product(_tangent, _N, _B);
      normalize(_B);
    }
    else
    {
      cross_product(_B, _tangent, _N);
      normalize(_N);
      cross_product(_tangent, _N, _B);
      normalize(_B);
    }

    point v0;
    v0.x = pointList[i].x + alpha * (_B[0] - _N[0]);
    v0.y = pointList[i].y + alpha * (_B[1] - _N[1]);
    v0.z = pointList[i].z + alpha * (_B[2] - _N[2]);
    trackList.push_back(v0);

    point v1;
    v1.x = pointList[i].x + alpha * (_B[0] - 0.5 * _N[0]);
    v1.y = pointList[i].y + alpha * (_B[1] - 0.5 * _N[1]);
    v1.z = pointList[i].z + alpha * (_B[2] - 0.5 * _N[2]);
    trackList.push_back(v1);

    point v2;
    v2.x = pointList[i].x + alpha * (0.2 * _B[0] - 0.5 * _N[0]);
    v2.y = pointList[i].y + alpha * (0.2 * _B[1] - 0.5 * _N[1]);
    v2.z = pointList[i].z + alpha * (0.2 * _B[2] - 0.5 * _N[2]);
    trackList.push_back(v2);

    point v3;
    v3.x = pointList[i].x + alpha * (0.2 * _B[0] + _N[0]);
    v3.y = pointList[i].y + alpha * (0.2 * _B[1] + _N[1]);
    v3.z = pointList[i].z + alpha * (0.2 * _B[2] + _N[2]);
    trackList.push_back(v3);

    point v4;
    v4.x = pointList[i].x + alpha * (-0.2 * _B[0] + _N[0]);
    v4.y = pointList[i].y + alpha * (-0.2 * _B[1] + _N[1]);
    v4.z = pointList[i].z + alpha * (-0.2 * _B[2] + _N[2]);
    trackList.push_back(v4);

    point v5;
    v5.x = pointList[i].x + alpha * (0.2 * -_B[0] - 0.5 * _N[0]);
    v5.y = pointList[i].y + alpha * (0.2 * -_B[1] - 0.5 * _N[1]);
    v5.z = pointList[i].z + alpha * (0.2 * -_B[2] - 0.5 * _N[2]);
    trackList.push_back(v5);

    point v6;
    v6.x = pointList[i].x + alpha * (-_B[0] - 0.5 * _N[0]);
    v6.y = pointList[i].y + alpha * (-_B[1] - 0.5 * _N[1]);
    v6.z = pointList[i].z + alpha * (-_B[2] - 0.5 * _N[2]);
    trackList.push_back(v6);

    point v7;
    v7.x = pointList[i].x + alpha * (-_B[0] - _N[0]);
    v7.y = pointList[i].y + alpha * (-_B[1] - _N[1]);
    v7.z = pointList[i].z + alpha * (-_B[2] - _N[2]);
    trackList.push_back(v7);

    point v8;
    v8.x = pointList[i].x + alpha * (5 * _B[0] - _N[0]);
    v8.y = pointList[i].y + alpha * (5 * _B[1] - _N[1]);
    v8.z = pointList[i].z + alpha * (5 * _B[2] - _N[2]);
    trackList2.push_back(v8);

    point v9;
    v9.x = pointList[i].x + alpha * (5 * _B[0] - 0.5 * _N[0]);
    v9.y = pointList[i].y + alpha * (5 * _B[1] - 0.5 * _N[1]);
    v9.z = pointList[i].z + alpha * (5 * _B[2] - 0.5 * _N[2]);
    trackList2.push_back(v9);

    point v10;
    v10.x = pointList[i].x + alpha * (4.2 * _B[0] - 0.5 * _N[0]);
    v10.y = pointList[i].y + alpha * (4.2 * _B[1] - 0.5 * _N[1]);
    v10.z = pointList[i].z + alpha * (4.2 * _B[2] - 0.5 * _N[2]);
    trackList2.push_back(v10);

    point v11;
    v11.x = pointList[i].x + alpha * (4.2 * _B[0] + _N[0]);
    v11.y = pointList[i].y + alpha * (4.2 * _B[1] + _N[1]);
    v11.z = pointList[i].z + alpha * (4.2 * _B[2] + _N[2]);
    trackList2.push_back(v11);

    point v12;
    v12.x = pointList[i].x + alpha * (3.8 * _B[0] + _N[0]);
    v12.y = pointList[i].y + alpha * (3.8 * _B[1] + _N[1]);
    v12.z = pointList[i].z + alpha * (3.8 * _B[2] + _N[2]);
    trackList2.push_back(v12);

    point v13;
    v13.x = pointList[i].x + alpha * (3.8 * _B[0] - 0.5 * _N[0]);
    v13.y = pointList[i].y + alpha * (3.8 * _B[1] - 0.5 * _N[1]);
    v13.z = pointList[i].z + alpha * (3.8 * _B[2] - 0.5 * _N[2]);
    trackList2.push_back(v13);

    point v14;
    v14.x = pointList[i].x + alpha * (3 * _B[0] - 0.5 * _N[0]);
    v14.y = pointList[i].y + alpha * (3 * _B[1] - 0.5 * _N[1]);
    v14.z = pointList[i].z + alpha * (3 * _B[2] - 0.5 * _N[2]);
    trackList2.push_back(v14);

    point v15;
    v15.x = pointList[i].x + alpha * (3 * _B[0] - _N[0]);
    v15.y = pointList[i].y + alpha * (3 * _B[1] - _N[1]);
    v15.z = pointList[i].z + alpha * (3 * _B[2] - _N[2]);
    trackList2.push_back(v15);

    if (pointList[i].z > pointList[i - 1].z && pointList[i].z > pointList[i + 1].z)
    {
      supportList.push_back(v0);
      supportList.push_back(v7);

      supportList2.push_back(v8);
      supportList2.push_back(v15);
    }
  }
}

void drawTrack()
{
  glEnable(GL_TEXTURE_2D);
  for (int i = 0; i < trackList.size() - 8; i += 8)
  {
    glBindTexture(GL_TEXTURE_2D, trackTex);

    glBegin(GL_QUADS);
    glTexCoord2f(0.0, 0.0);
    glVertex3d(trackList[i].x, trackList[i].y, trackList[i].z);
    glTexCoord2f(0.0, 1.0);
    glVertex3d(trackList[i + 1].x, trackList[i + 1].y, trackList[i + 1].z);
    glTexCoord2f(1.0, 1.0);
    glVertex3d(trackList[i + 9].x, trackList[i + 9].y, trackList[i + 9].z);
    glTexCoord2f(1.0, 0.0);
    glVertex3d(trackList[i + 8].x, trackList[i + 8].y, trackList[i + 8].z);
    glEnd();

    glBegin(GL_QUADS);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList[i + 1].x, trackList[i + 1].y, trackList[i + 1].z);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList[i + 2].x, trackList[i + 2].y, trackList[i + 2].z);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList[i + 10].x, trackList[i + 10].y, trackList[i + 10].z);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList[i + 9].x, trackList[i + 9].y, trackList[i + 9].z);
    glEnd();

    glBegin(GL_QUADS);
    glTexCoord2f(0.0, 0.0);
    glVertex3d(trackList[i + 7].x, trackList[i + 7].y, trackList[i + 7].z);
    glTexCoord2f(0.0, 1.0);
    glVertex3d(trackList[i + 6].x, trackList[i + 6].y, trackList[i + 6].z);
    glTexCoord2f(1.0, 1.0);
    glVertex3d(trackList[i + 14].x, trackList[i + 14].y, trackList[i + 14].z);
    glTexCoord2f(1.0, 0.0);
    glVertex3d(trackList[i + 15].x, trackList[i + 15].y, trackList[i + 15].z);
    glEnd();

    glBegin(GL_QUADS);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList[i + 6].x, trackList[i + 6].y, trackList[i + 6].z);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList[i + 5].x, trackList[i + 5].y, trackList[i + 5].z);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList[i + 13].x, trackList[i + 13].y, trackList[i + 13].z);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList[i + 14].x, trackList[i + 14].y, trackList[i + 14].z);
    glEnd();

    glBindTexture(GL_TEXTURE_2D, track2Tex);

    glBegin(GL_QUADS);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList[i + 2].x, trackList[i + 2].y, trackList[i + 2].z);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList[i + 3].x, trackList[i + 3].y, trackList[i + 3].z);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList[i + 11].x, trackList[i + 11].y, trackList[i + 11].z);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList[i + 10].x, trackList[i + 10].y, trackList[i + 10].z);
    glEnd();

    glBegin(GL_QUADS);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList[i + 5].x, trackList[i + 5].y, trackList[i + 5].z);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList[i + 4].x, trackList[i + 4].y, trackList[i + 4].z);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList[i + 12].x, trackList[i + 12].y, trackList[i + 12].z);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList[i + 13].x, trackList[i + 13].y, trackList[i + 13].z);
    glEnd();

    glBegin(GL_QUADS);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList[i + 3].x, trackList[i + 3].y, trackList[i + 3].z);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList[i + 4].x, trackList[i + 4].y, trackList[i + 4].z);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList[i + 12].x, trackList[i + 12].y, trackList[i + 12].z);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList[i + 11].x, trackList[i + 11].y, trackList[i + 11].z);
    glEnd();

    // render the second track

    glBindTexture(GL_TEXTURE_2D, trackTex);

    glBegin(GL_QUADS);
    glTexCoord2f(0.0, 0.0);
    glVertex3d(trackList2[i].x, trackList2[i].y, trackList2[i].z);
    glTexCoord2f(0.0, 1.0);
    glVertex3d(trackList2[i + 1].x, trackList2[i + 1].y, trackList2[i + 1].z);
    glTexCoord2f(1.0, 1.0);
    glVertex3d(trackList2[i + 9].x, trackList2[i + 9].y, trackList2[i + 9].z);
    glTexCoord2f(1.0, 0.0);
    glVertex3d(trackList2[i + 8].x, trackList2[i + 8].y, trackList2[i + 8].z);
    glEnd();

    glBegin(GL_QUADS);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList2[i + 1].x, trackList2[i + 1].y, trackList2[i + 1].z);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList2[i + 2].x, trackList2[i + 2].y, trackList2[i + 2].z);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList2[i + 10].x, trackList2[i + 10].y, trackList2[i + 10].z);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList2[i + 9].x, trackList2[i + 9].y, trackList2[i + 9].z);
    glEnd();

    glBegin(GL_QUADS);
    glTexCoord2f(0.0, 0.0);
    glVertex3d(trackList2[i + 7].x, trackList2[i + 7].y, trackList2[i + 7].z);
    glTexCoord2f(0.0, 1.0);
    glVertex3d(trackList2[i + 6].x, trackList2[i + 6].y, trackList2[i + 6].z);
    glTexCoord2f(1.0, 1.0);
    glVertex3d(trackList2[i + 14].x, trackList2[i + 14].y, trackList2[i + 14].z);
    glTexCoord2f(1.0, 0.0);
    glVertex3d(trackList2[i + 15].x, trackList2[i + 15].y, trackList2[i + 15].z);
    glEnd();

    glBegin(GL_QUADS);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList2[i + 6].x, trackList2[i + 6].y, trackList2[i + 6].z);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList2[i + 5].x, trackList2[i + 5].y, trackList2[i + 5].z);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList2[i + 13].x, trackList2[i + 13].y, trackList2[i + 13].z);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList2[i + 14].x, trackList2[i + 14].y, trackList2[i + 14].z);
    glEnd();

    glBindTexture(GL_TEXTURE_2D, track2Tex);

    glBegin(GL_QUADS);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList2[i + 2].x, trackList2[i + 2].y, trackList2[i + 2].z);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList2[i + 3].x, trackList2[i + 3].y, trackList2[i + 3].z);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList2[i + 11].x, trackList2[i + 11].y, trackList2[i + 11].z);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList2[i + 10].x, trackList2[i + 10].y, trackList2[i + 10].z);
    glEnd();

    glBegin(GL_QUADS);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList2[i + 5].x, trackList2[i + 5].y, trackList2[i + 5].z);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList2[i + 4].x, trackList2[i + 4].y, trackList2[i + 4].z);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList2[i + 12].x, trackList2[i + 12].y, trackList2[i + 12].z);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList2[i + 13].x, trackList2[i + 13].y, trackList2[i + 13].z);
    glEnd();

    glBegin(GL_QUADS);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList2[i + 3].x, trackList2[i + 3].y, trackList2[i + 3].z);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList2[i + 4].x, trackList2[i + 4].y, trackList2[i + 4].z);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList2[i + 12].x, trackList2[i + 12].y, trackList2[i + 12].z);
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(trackList2[i + 11].x, trackList2[i + 11].y, trackList2[i + 11].z);
    glEnd();
  }
  glDisable(GL_TEXTURE_2D);
}

void drawSupport()
{
  glBindTexture(GL_TEXTURE_2D, supportTex);
  glEnable(GL_TEXTURE_2D);
  int supportPos = 0;
  while (supportPos < supportList.size() - 1)
  {
    glBegin(GL_QUADS);
    glTexCoord2f(0.0, 0.0);
    glVertex3d(supportList[supportPos].x, supportList[supportPos].y, supportList[supportPos].z);
    glTexCoord2f(0.0, 1.0);
    glVertex3d(supportList[supportPos].x, supportList[supportPos].y, 5.0);
    glTexCoord2f(1.0, 1.0);
    glVertex3d(supportList[supportPos + 1].x, supportList[supportPos + 1].y, 5.0);
    glTexCoord2f(1.0, 0.0);
    glVertex3d(supportList[supportPos + 1].x, supportList[supportPos + 1].y, supportList[supportPos + 1].z);
    glEnd();

    glBegin(GL_QUADS);
    glTexCoord2f(0.0, 0.0);
    glVertex3d(supportList2[supportPos].x, supportList2[supportPos].y, supportList2[supportPos].z);
    glTexCoord2f(0.0, 1.0);
    glVertex3d(supportList2[supportPos].x, supportList2[supportPos].y, 5.0);
    glTexCoord2f(1.0, 1.0);
    glVertex3d(supportList2[supportPos + 1].x, supportList2[supportPos + 1].y, 5.0);
    glTexCoord2f(1.0, 0.0);
    glVertex3d(supportList2[supportPos + 1].x, supportList2[supportPos + 1].y, supportList2[supportPos + 1].z);
    glEnd();

    supportPos += 2;
  }
  glDisable(GL_TEXTURE_2D);
}

/* moves the camera along the splines */
void moveCamera()
{
  cameraPos[0] = pointList[cameraPoint].x;
  cameraPos[1] = pointList[cameraPoint].y;
  cameraPos[2] = pointList[cameraPoint].z;

  tangent[0] = pointList[cameraPoint + 1].x - pointList[cameraPoint - 1].x;
  tangent[1] = pointList[cameraPoint + 1].y - pointList[cameraPoint - 1].y;
  tangent[2] = pointList[cameraPoint + 1].z - pointList[cameraPoint - 1].z;

  normalize(tangent);

  // Initialization
  if (cameraPoint == 1)
  {
    cross_product(tangent, arbitrary_vector, N);
    normalize(N);
    cross_product(tangent, N, B);
    normalize(B);
  }
  else
  {
    cross_product(B, tangent, N);
    normalize(N);
    cross_product(tangent, N, B);
    normalize(B);
  }

  cameraPoint += 2;
  if (cameraPoint >= pointList.size() - 2)
  {
    cameraPoint = 1;
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

  g_pSkyPic = jpeg_read("sky_texture.jpeg", NULL);
  g_pGroundPic = jpeg_read("ground_texture.jpeg", NULL);
  g_pTrackPic = jpeg_read("orange.jpg", NULL);
  g_pSupportPic = jpeg_read("blue.jpeg", NULL);
  g_pTrack2Pic = jpeg_read("track_texture.jpg", NULL);

  if (!g_pSkyPic || !g_pGroundPic)
  {
    printf("error reading texture files.\n");
    exit(1);
  }

  initTexture(&skyTex, g_pSkyPic);
  initTexture(&groundTex, g_pGroundPic);
  initTexture(&trackTex, g_pTrackPic);
  initTexture(&supportTex, g_pSupportPic);
  initTexture(&track2Tex, g_pTrack2Pic);
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
  // gluLookAt(0.0, 0.0, 6.5, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);

  moveCamera();

  double offset = 0.1;

  gluLookAt(cameraPos[0] + offset * N[0] + 0.02 * B[0], cameraPos[1] + offset * N[1] + 0.02 * B[1], cameraPos[2] + offset * N[2] + 0.02 * B[2], cameraPos[0] + tangent[0], cameraPos[1] + tangent[1], cameraPos[2] + tangent[2], N[0], N[1], N[2]);

  // sets translation, rotation, and scale
  glTranslatef(g_vLandTranslate[0], g_vLandTranslate[1], g_vLandTranslate[2]);
  glRotatef(g_vLandRotate[0], 1, 0, 0);
  glRotatef(g_vLandRotate[1], 0, 1, 0);
  glRotatef(g_vLandRotate[2], 0, 0, 1);
  glScalef(g_vLandScale[0], g_vLandScale[1], g_vLandScale[2]);

  drawGround();
  // drawSplines();
  drawTrack();
  //  drawSupport();

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
  gluPerspective(60, aspect, 0.01, 500.0);
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

  createSplines();

  createTrack();

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
