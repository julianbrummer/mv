#ifndef MV_H
#define MV_H

#include <QMainWindow>
#include <QGridLayout>
#include <QFrame>
#include <QtCore>
#include <QLocale>
#include <QInputDialog>

#include <QtOpenGL/QGLWidget>
#include <QtOpenGL/QGLFunctions>
#include <QtOpenGL/QGLShaderProgram>
#include <QOpenGLFunctions_4_2_Core>

#include <vector>
#include <unordered_map>
#include <iostream>
#include <Eigen/Dense>
#include "dualconturing.h"

using namespace Eigen;

class MyGLWidget;

class CGMainWindow : public QMainWindow {
    Q_OBJECT

public:

    CGMainWindow (QWidget* parent = 0);
    ~CGMainWindow ();
    MyGLWidget *ogl;

public slots:
    void loadModel();
    void loadTrack();
    void dualConturing();

protected:

    void keyPressEvent(QKeyEvent*);
};


class MyGLWidget : public QGLWidget, public QOpenGLFunctions_4_2_Core {
    Q_OBJECT

public:

    MyGLWidget(CGMainWindow*,QWidget*);
    bool initShaderProgram(const char *vname, const char *fname, QGLShaderProgram& program);
    void initializeGL();

    bool pick(int,int,QVector3D&);
    void mouseToTrackball(int,int,int,int,QVector3D&);
    QQuaternion trackball(const QVector3D&,const QVector3D&);
    void pickLine(int,int,QVector3D&,QVector3D&);

    template <typename VBO_DATA>
    GLuint initVBO(const std::vector<VBO_DATA> &data);
    GLuint initIBO(const std::vector<uint>& iboData);
    void initSolidCubeVBO();
    void initSolidSphereVBO();
    void initSolidCylinderVBO();
    void initAxisVBO(float size);
    void refineSolidSphere(const std::vector<QVector3D>&, std::vector<QVector3D>&);
    void initTrianglesVBO(const std::vector<QVector3D>&); 
    void initVoxelVBO(const std::vector<bool>&,const uint32_t[3],const QVector3D&,const qreal&);
    void initCellVBO(const vector<Vertex>& vertices);
    void initEdgeVBO(const vector<Vertex>& vertices);
    void initGridVBO(const vector<Vertex> &vertices);
    void initDCVerticesVBO(const vector<Vertex>& vertices);
    void initDCMeshVBO(const vector<Vertex>& vertices);
    void initDCMeshIBO(const vector<uint>& indices);

    void snapshotDualConturing(bool front_faces);
    void snapshot(QGLShaderProgram &program);

    void createAll();
    void copyEdgeData(std::vector<std::vector<GLuint> > &edgeData);
    void copyGridData(std::vector<bool>& gridData);

    QVector3D min, max, center;
    Vector3f origin;
    QQuaternion qNow;
    qreal zoom;
    bool wireframe;
    bool showModel;

    QVector3D modelCenter; // the center in object space (before translating e.g by mouse wheel)
    qreal scaling; // initial scaling of the model (before zooming)
    bool showEdgeCuts, showCells, showGrid, showDCVertices, showDCMesh, showAxis;
    uint res;

    qreal timestep;
    std::vector<QVector3D> triangles,pickTriangles;

    GLuint vboTrianglesId,vboSolidCubeId,vboSolidSphereId,vboSolidCylinderId,vboAxisId,
           vboVoxelId,vboGridId,vboDCVerticesId,vboDCMeshId,iboDCMeshId,vboEdgeId,vboCellId;

    int vboTrianglesSize,vboSolidCubeSize,vboSolidSphereSize,vboSolidCylinderSize,vboAxisSize,
        vboVoxelSize,vboGridSize,vboDCVerticesSize,vboDCMeshSize,iboDCMeshSize,vboEdgeSize,vboCellSize;

    std::vector<QMatrix4x4> trafo;
    int trafo_cnt;

    QMatrix4x4 transform(int);
    void aabb(std::vector<QVector3D>&,qreal&,qreal&,qreal&,qreal&,qreal&,qreal&,const QMatrix4x4& M = QMatrix4x4());

    QVector3D pickPoint;
    bool pointPicked;

protected:

    void paintGL();
    void resizeGL(int,int);

    void mouseMoveEvent(QMouseEvent*);
    void mousePressEvent(QMouseEvent*);
    void mouseReleaseEvent(QMouseEvent*);
    void wheelEvent(QWheelEvent*);
    void drawSolidSphere(const QVector3D&,qreal);
    void drawSolidCylinder(const QVector3D&,const QVector3D&,qreal);
    void drawVBO(int id, int size, QGLShaderProgram &program, GLenum mode, bool normal, bool color);

    CGMainWindow *main;
    int oldX,oldY,button;

private:
    QGLShaderProgram program, programColor;
    QGLShaderProgram snapshotDCProgram;
    QMatrix4x4 projection,modelView;
    int width, height;
    GLuint edges[3];
    dc::MeshGenerator meshGenerator;
};

#endif
