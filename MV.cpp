#include <QApplication>
#include <QMenu>
#include <QMenuBar>
#include <QStatusBar>
#include <QFileDialog>
#include <QKeyEvent>
#include <QMessageBox>
#include <QHBoxLayout>


#include "MV.h"

#include <algorithm>
#include <stack>
#include <queue>
#include <set>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <limits>
#include <thread>
#include <atomic>
#include <cstring>
#include <unistd.h>

#include <QVector3D>

CGMainWindow *w;

CGMainWindow::CGMainWindow (QWidget* parent) 
	: QMainWindow (parent) {
    resize (800,800);

    // Create a menu
    QMenu *file = new QMenu("&File",this);
    file->addAction ("Load model", this, SLOT(loadModel()), Qt::CTRL+Qt::Key_L);
    file->addAction ("Load track", this, SLOT(loadTrack()), Qt::CTRL+Qt::Key_T);
    file->addAction ("Dual Conturing", this, SLOT(dualConturing()), Qt::CTRL+Qt::Key_D);
    file->addAction ("Quit", qApp, SLOT(quit()), Qt::CTRL+Qt::Key_Q);

    menuBar()->addMenu(file);

    QMenu *view = new QMenu("&Extra",this);
    view->addAction ("Quit", qApp, SLOT(quit()), Qt::CTRL+Qt::Key_Q);
    menuBar()->addMenu(view);

    // Create a nice frame to put around the OpenGL widget
    QFrame* f = new QFrame (this);
    f->setFrameStyle(QFrame::Sunken | QFrame::Panel);
    f->setLineWidth(2);

    // Create our OpenGL widget
    ogl = new MyGLWidget (this,f);

    // Put the GL widget inside the frame
    QHBoxLayout* layout = new QHBoxLayout();
    layout->addWidget(ogl);
    layout->setMargin(0);
    f->setLayout(layout);

    setCentralWidget(f);

    statusBar()->showMessage("Ready",1000);
}

CGMainWindow::~CGMainWindow () {}

void CGMainWindow::keyPressEvent(QKeyEvent* event) {
    switch(event->key()) {
    case Qt::Key_Plus: break;
    case Qt::Key_Minus: break;
    case Qt::Key_Right: ogl->trafo_cnt += 1; if (ogl->trafo_cnt >= (int) ogl->trafo.size()) ogl->trafo_cnt = ogl->trafo.size()-1; break;
    case Qt::Key_Left: ogl->trafo_cnt -= 1; if (ogl->trafo_cnt < 0) ogl->trafo_cnt = 0; break;
    case Qt::Key_Up: ogl->trafo_cnt += 10; if (ogl->trafo_cnt >= (int) ogl->trafo.size()) ogl->trafo_cnt = ogl->trafo.size()-1; break;
    case Qt::Key_Down: ogl->trafo_cnt -= 10; if (ogl->trafo_cnt < 0) ogl->trafo_cnt = 0; break;
    case Qt::Key_PageUp: ogl->trafo_cnt += 1000; if (ogl->trafo_cnt >= (int) ogl->trafo.size()) ogl->trafo_cnt = ogl->trafo.size()-1; break;
    case Qt::Key_PageDown: ogl->trafo_cnt -= 1000; if (ogl->trafo_cnt < 0) ogl->trafo_cnt = 0; break;
    case Qt::Key_M: ogl->showModel = !ogl->showModel; break;
    case Qt::Key_E: ogl->showEdgeCuts = !ogl->showEdgeCuts; break;
    case Qt::Key_C: ogl->showCells = !ogl->showCells; break;
    case Qt::Key_V: ogl->showDCVertices = !ogl->showDCVertices; break;
    case Qt::Key_G: ogl->showGrid = !ogl->showGrid; break;
    case Qt::Key_D: ogl->showDCMesh = !ogl->showDCMesh; break;
    case Qt::Key_W: ogl->wireframe = !ogl->wireframe; break;
    }

    ogl->updateGL();
}

bool intersectLine(const QVector3D& a,const QVector3D& b,const QVector3D& c,const QVector3D& p,const QVector3D& q,QVector3D& s,qreal& d,int dir) {
    QVector3D n = QVector3D::normal(a,b,c);
    if (dir != 0) {
        qreal npq = QVector3D::dotProduct(n,p-q);
        if ((npq > 0) && (dir < 0)) return false;
        if ((npq < 0) && (dir > 0)) return false;
    }
    d = QVector3D::dotProduct(n,q-p);
    if (d == 0) return false;
    d = QVector3D::dotProduct(n,a-p)/d;
    s = p+(q-p)*d;

    if (QVector3D::dotProduct(n,QVector3D::crossProduct(b-a,s-a)) < 0) return false;
    if (QVector3D::dotProduct(n,QVector3D::crossProduct(c-b,s-b)) < 0) return false;
    if (QVector3D::dotProduct(n,QVector3D::crossProduct(a-c,s-c)) < 0) return false;

    return true;
}

void getFromOffFile(std::vector<QVector3D>& triangles, const char *filename) {
    std::ifstream instream(filename);
    if (!instream) {
        std::cerr << "file does not exist!" << std::endl;
        return;
    }
    char buffer[80];
    int nv,nf,ne;
    instream.getline(buffer,80);
    instream >> nv >> nf >> ne;

    QVector3D* vertex = new QVector3D[nv];

    for(int i=0;i<nv;i++) {
        qreal x,y,z;
        instream >> x >> y >> z;
        vertex[i] = QVector3D(x,y,z);
    }

    for(int i=0;i<nf;i++) {
        int i0,i1,i2,i3;
        //double c0,c1,c2,c3;
        //instream >> i0 >> i1 >> i2 >> i3 >> c0 >> c1 >> c2 >> c3;
        instream >> i0 >> i1 >> i2 >> i3;
        triangles.push_back(vertex[i1]);
        triangles.push_back(vertex[i2]);
        triangles.push_back(vertex[i3]);
    }

    delete[] vertex;
    instream.close();
}

void getFromStlFile(std::vector<QVector3D>& triangles, const char *filename) {
    std::ifstream instream(filename,std::ios::binary);
    if (!instream) {
        std::cerr << "file does not exist!" << std::endl;
        return;
    }

    instream.seekg( 80, std::ios_base::beg ); // skip ascii header
    int trinum = 0;
    instream.read((char*) &trinum, 4 ); // number of triangles
    float tmp;
    for(int k = 0; k < trinum; k++) {
        for(int i=0;i < 3 ; i++ )
            instream.read( (char*) &tmp, 4 );
        for(int i = 0; i < 3; i++ ) {
		qreal v[3];
            for(int j = 0 ; j < 3 ; j++) {
                instream.read( (char*) &tmp, 4 );
                v[j] = tmp;
            }
            triangles.push_back(1000.0*QVector3D(v[0],v[1],v[2]));
        }
        instream.read( (char*) &tmp, 2);
    }

    instream.close();
}

void getFromVdaFile( std::vector<QMatrix4x4>& trafo, const char* filename, qreal& timestep ) {
    std::setlocale(LC_NUMERIC,"C");

    std::ifstream vdafile( filename );
    if (!vdafile) {
        std::cerr << "getFromVdaFile: Cannot open vda-file" << std::endl;
        return;
    }

    trafo.clear();

    std::string s,t;
    size_t pos1,pos2,pos3;

    getline(vdafile,s);
    getline(vdafile,s);
    getline(vdafile,s);

    if (s.find("DT")) {
            pos1 = s.find(".");
            pos1 = s.find(".",pos1+1);
            t = s.substr(pos1,pos1+5);
            timestep = atof(t.data());
    }

    getline(vdafile,s);

    QMatrix4x4 M;
    M.setToIdentity();

    int n = 0;
    while (!vdafile.eof()) {
            getline(vdafile,s);

            if (s.find("TMAT") == std::string::npos) {
                    // std::cout << "end of file" << std::endl;
                    break;
            }

            for(int i=0;i<3;i++) {
                    getline(vdafile,s);
                    pos1 = s.find(",");
                    t = s.substr(0,pos1);
                    M(i,0) = atof(t.data());
                    pos2 = s.find(",",pos1+1);
                    t = s.substr(pos1+1,pos2-pos1-1);
                    M(i,1) = atof(t.data());
                    pos3 = s.find(",",pos2+1);
                    t = s.substr(pos2+1,pos3-pos2-1);
                    M(i,2) = atof(t.data());
            }

            getline(vdafile,s);
            pos1 = s.find(",");
            t = s.substr(0,pos1);
            M(0,3) = atof(t.data());
            pos2 = s.find(",",pos1+1);
            t = s.substr(pos1+1,pos2-pos1-1);
            M(1,3) = atof(t.data());
            pos3 = s.find(" ",pos2+1);
            t = s.substr(pos2+1,pos3-pos2-1);
            M(2,3) = atof(t.data());

            trafo.push_back(M);
            n++;
    }

    vdafile.close();

    std::cout << "Loaded " << filename << " with " << trafo.size() << " transformations" << std::endl;
}

void writeToStlFile(std::vector<QVector3D>& T, const char *filename) {
	char buffer[80];
	for(int i=0;i<80;i++)
		buffer[i] = ' ';
	buffer[0] = 'V';
	buffer[1] = 'C';
	buffer[2] = 'G';

        std::ofstream outstream(filename,std::ofstream::binary);
	outstream.write(buffer,80);
	unsigned int n = T.size()/3;
	outstream.write((char*) &n,4);

	float f;
	unsigned short s = 0;

	for(size_t j=0;j<n;j++) {
		const QVector3D& a = T[3*j+0];
		const QVector3D& b = T[3*j+1];
		const QVector3D& c = T[3*j+2];
		QVector3D n = QVector3D::normal(a,b,c);
		f = n.x(); outstream.write((char*) &f,4);
		f = n.y(); outstream.write((char*) &f,4);
		f = n.z(); outstream.write((char*) &f,4);

		f = a.x(); outstream.write((char*) &f,4);
		f = a.y(); outstream.write((char*) &f,4);
		f = a.z(); outstream.write((char*) &f,4);

		f = b.x(); outstream.write((char*) &f,4);
		f = b.y(); outstream.write((char*) &f,4);
		f = b.z(); outstream.write((char*) &f,4);

		f = c.x(); outstream.write((char*) &f,4);
		f = c.y(); outstream.write((char*) &f,4);
		f = c.z(); outstream.write((char*) &f,4);

		outstream.write((char*) &s,2);
	}

	outstream.close();
}

void writeToStlFile(const std::vector<Vertex>& T, const char *filename) {
    char buffer[80];
    for(int i=0;i<80;i++)
        buffer[i] = ' ';
    buffer[0] = 'V';
    buffer[1] = 'C';
    buffer[2] = 'G';

        std::ofstream outstream(filename,std::ofstream::binary);
    outstream.write(buffer,80);
    unsigned int n = T.size()/3;
    outstream.write((char*) &n,4);

    float f;
    unsigned short s = 0;

    for(size_t j=0;j<n;j++) {
        const Vector3f& n = T[3*j+0].normal;
        const Vector3f& a = T[3*j+0].position;
        const Vector3f& b = T[3*j+1].position;
        const Vector3f& c = T[3*j+2].position;
        f = n.x(); outstream.write((char*) &f,4);
        f = n.y(); outstream.write((char*) &f,4);
        f = n.z(); outstream.write((char*) &f,4);

        f = a.x(); outstream.write((char*) &f,4);
        f = a.y(); outstream.write((char*) &f,4);
        f = a.z(); outstream.write((char*) &f,4);

        f = b.x(); outstream.write((char*) &f,4);
        f = b.y(); outstream.write((char*) &f,4);
        f = b.z(); outstream.write((char*) &f,4);

        f = c.x(); outstream.write((char*) &f,4);
        f = c.y(); outstream.write((char*) &f,4);
        f = c.z(); outstream.write((char*) &f,4);

        outstream.write((char*) &s,2);
    }

    outstream.close();
}

void writeToOffFile(std::vector<QVector3D>& vertex, std::vector<int>& face,const char *filename) {
    std::ofstream offFile(filename);

    offFile << "OFF" << std::endl;
    offFile << vertex.size() << " " << face.size()/3 << " " << vertex.size()+face.size()/3-2 << std::endl;

    for(size_t i=0;i<vertex.size();i++) {
        const QVector3D& a = vertex[i];
        offFile << a.x() << " " << a.y() << " " << a.z() << std::endl;
    }

    for(size_t i=0;i<face.size()/3;i++)
        offFile << 3 << " " << face[3*i+0] << " " << face[3*i+1] << " " << face[3*i+2] << std::endl;

    offFile.close();
}

void CGMainWindow::loadTrack() {
// QString filename = QFileDialog::getOpenFileName(this, "Load track ...", QString(), "(*.vda)" );

// if (filename.isEmpty()) return;
// statusBar()->showMessage ("Loading track ...");

// if (filename.endsWith(".vda")) {
//     std::ifstream trackfile(filename.toLatin1());
// }

    getFromVdaFile(ogl->trafo,"tracks/track.vda",ogl->timestep);
}

void CGMainWindow::loadModel() {
/*
    QString filename = QFileDialog::getOpenFileName(this, "Load model ...", QString(), "(*.stl *.off)" );

    if (filename.isEmpty()) return;
    statusBar()->showMessage ("Loading model ...");

    if (filename.endsWith(".stl"))
        getFromStlFile(ogl->triangles,filename.toLatin1());

    if (filename.endsWith(".off"))
        getFromOffFile(ogl->triangles,filename.toLatin1());

    if (ogl->triangles.size() == 0) {
        statusBar()->showMessage ("Empty model.",3000);
        return;
    }
*/
    ogl->triangles.clear();
    //getFromStlFile(ogl->triangles,"Motor - Motorabdeckung und Luftfilter.stl");
    getFromOffFile(ogl->triangles,"models/test.off");

    ogl->pickTriangles = ogl->triangles;

    qreal r = 0.0;
    qreal x1,x2,y1,y2,z1,z2;
    //ogl->aabb(ogl->triangles,x1,y1,z1,x2,y2,z2,ogl->trafo[0]);
    ogl->aabb(ogl->triangles,x1,y1,z1,x2,y2,z2);

    x1 -= r;
    y1 -= r;
    z1 -= r;
    x2 += r;
    y2 += r;
    z2 += r;

    ogl->min = QVector3D(x1,y1,z1);
    ogl->max = QVector3D(x2,y2,z2);
    ogl->center = (ogl->min+ogl->max)/2;
    ogl->modelCenter = ogl->center;
    QVector3D extent = ogl->max - ogl->min;
    std::cout << "b = [" << ogl->min.x() << "," << ogl->max.x() << "] x [ "
                         << ogl->min.y() << "," << ogl->max.y() << "] x [ "
                         << ogl->min.z() << "," << ogl->max.z() << "]" << std::endl;
    std::cout << "e = " << extent.x() << " " << extent.y() << " " << extent.z() << std::endl;
    ogl->zoom = 1.5/std::max(std::max(extent.x(),extent.y()),extent.z());
    ogl->scaling = ogl->zoom;
    std::cout << "triangle count = " << ogl->triangles.size()/3 << std::endl;
    statusBar()->showMessage ("Loading model done.",3000);
    ogl->updateGL();
}

void CGMainWindow::dualConturing() {
    ogl->createAll();
}

MyGLWidget::MyGLWidget (CGMainWindow *mainwindow,QWidget* parent ) : QGLWidget (parent) {
    main = mainwindow;
}

bool MyGLWidget::initShaderProgram(const char *vname, const char *fname, QGLShaderProgram& program) {
    setlocale(LC_NUMERIC, "C");
    // shader
    if (!program.addShaderFromSourceFile(QGLShader::Vertex, vname))
        return false;

    if (!program.addShaderFromSourceFile(QGLShader::Fragment, fname))
        return false;

    if (!program.link())
        return false;

    if (!program.bind())
        return false;

    return true;
}

void MyGLWidget::initializeGL() {
    initializeOpenGLFunctions();
    initShaderProgram(":/shaders/vshader.glsl", ":/shaders/fshader.glsl", program);
    initShaderProgram(":/shaders/vpoints.glsl", ":/shaders/fpoints.glsl", programColor);
    initShaderProgram(":/shaders/vcalcShader.glsl", ":/shaders/fcalcShader.glsl", snapshotDCProgram);
    initSolidCubeVBO();
    initSolidSphereVBO();
    initSolidCylinderVBO();
    initAxisVBO(1.0f);

    qglClearColor(Qt::black);
    glPointSize(4.0);
    glEnable(GL_DEPTH_TEST);

    zoom = 1.0;

    qreal inf = std::numeric_limits<qreal>::infinity();
    min = QVector3D( inf, inf, inf);
    max = QVector3D(-inf,-inf,-inf);
    center = QVector3D(0,0,0);
    pointPicked = false;
    //main->loadTrack();
    main->loadModel();

    initTrianglesVBO(triangles);
    wireframe = false;
    showModel = true;
    showDCVertices = false;
    showDCMesh = false;
    showGrid = false;
    showEdgeCuts = false;
    showCells = false;
    showAxis = true;


    res = 129;
    glGenTextures(3, edges);

}

void MyGLWidget::aabb(std::vector<QVector3D>& T,qreal& x1,qreal& y1,qreal& z1,qreal& x2,qreal& y2,qreal& z2,const QMatrix4x4& M) {
    qreal inf = std::numeric_limits<qreal>::infinity();

    x1 = y1 = z1 =  inf;
    x2 = y2 = z2 = -inf;

    for(size_t i=0;i<T.size();i++) {
        QVector3D v = M * T[i];
        if (v.x() < x1) x1 = v.x();
        if (v.x() > x2) x2 = v.x();
        if (v.y() < y1) y1 = v.y();
        if (v.y() > y2) y2 = v.y();
        if (v.z() < z1) z1 = v.z();
        if (v.z() > z2) z2 = v.z();
    }
}

QMatrix4x4 MyGLWidget::transform(int trackIdx) {
    return trafo[trackIdx];
}

void MyGLWidget::paintGL() {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    modelView.setToIdentity();
    modelView.translate(0,0,-3);
    modelView.rotate(qNow);
    modelView.scale(zoom,zoom,zoom);
    modelView.translate(-center);
    program.bind();
/*
    if (pointPicked) {
        program.setUniformValue("uColor", QVector4D(1.0,1.0,0.0,1.0));
        drawSolidSphere(pickPoint,5.0);
    }
*/

    if (showModel) {
        QMatrix4x4 M = modelView;
        //M.scale(0.5,0.5,0.5);
        //M.rotate(32, 1, 0.5, 0);

        //QMatrix4x4 M = modelView * trafo[trafo_cnt];

        program.setUniformValue("uPMat", projection);

        program.setUniformValue("uMVMat", M);
        program.setUniformValue("uNMat", M.normalMatrix());
        program.setUniformValue("uColor", QVector4D(0.75,0.75,0.75,1.0));
        //drawVBO(vboSolidCubeId, vboSolidCubeSize, program, GL_TRIANGLES, true, false);
        drawVBO(vboTrianglesId, vboTrianglesSize, program, GL_TRIANGLES, true, false);
        //drawVBO(vboSolidSphereId, vboSolidSphereSize, program, GL_TRIANGLES, true, false);
    }

    QMatrix4x4 M = QMatrix4x4();
    M.translate(0,0,-3);
    M.rotate(qNow);
    M.scale(zoom, zoom, zoom);
    //M.translate(-center);
    M.translate(modelCenter-center);
    M.scale(1.0/scaling, 1.0/scaling, 1.0/scaling);

    if (showDCMesh) {
        if (wireframe)
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        program.setUniformValue("uPMat", projection);
        program.setUniformValue("uMVMat", M);
        program.setUniformValue("uNMat", M.normalMatrix());
        program.setUniformValue("uColor", QVector4D(0.0,0.0,0.75,1.0));
        drawVBO(vboDCMeshId, vboDCMeshSize, program, GL_TRIANGLES, true, true);
    }

    if (showDCVertices || showGrid || showEdgeCuts || showCells || showAxis) {
        programColor.bind();
        programColor.setUniformValue("uPMat", projection);
        programColor.setUniformValue("uMVMat", M);
        if (showDCVertices)
            drawVBO(vboDCVerticesId, vboDCVerticesSize, programColor, GL_POINTS, true, true);
        if (showGrid)
            drawVBO(vboGridId, vboGridSize, programColor, GL_POINTS, true, true);
        if (showEdgeCuts)
            drawVBO(vboEdgeId, vboEdgeSize, programColor, GL_POINTS, true, true);
        if (showCells)
            drawVBO(vboCellId, vboCellSize, programColor, GL_LINES, true, true);
        if (showAxis)
            drawVBO(vboAxisId, vboAxisSize, programColor, GL_LINES, false, true);
    }






}

void MyGLWidget::resizeGL(int w, int h) {
    width = w;
    height = h;
    glViewport(0,0,width,height);
    projection.setToIdentity();
    if (width > height) {
        qreal ratio = width/(qreal) height;
        //projection.ortho(-ratio,ratio,-1.0,1.0,-10.0,10.0);
        projection.perspective(45.0f,ratio,0.1,100);
    } else {
        qreal ratio = height/(qreal) width;
        //projection.ortho(-1.0,1.0,-ratio,ratio,-10.0,10.0);
        projection.perspective(45.0f,ratio,0.1,100);
    }
}

void MyGLWidget::pickLine(int x,int y,QVector3D& u,QVector3D& v) {
    modelView.setToIdentity();
    modelView.translate(0,0,-3);
    modelView.rotate(qNow);
    modelView.scale(zoom,zoom,zoom);
    modelView.translate(-center);

    QMatrix4x4 T = (projection*modelView).inverted();

    qreal xn = (2.0*x)/width-1.0;
    qreal yn = 1.0-(2.0*y)/height;

    u = T * QVector3D(xn,yn,0.0);
    v = T * QVector3D(xn,yn,1.0);
}

bool MyGLWidget::pick(int x,int y,QVector3D& pickPoint) {
    QVector3D u,v;
    pickLine(x,y,u,v);

    qreal inf = std::numeric_limits<qreal>::infinity();
    qreal dmin = inf;

    for(size_t j=0;j<pickTriangles.size();j+=3) {
        const QVector3D& a = pickTriangles[j+0];
        const QVector3D& b = pickTriangles[j+1];
        const QVector3D& c = pickTriangles[j+2];
        QVector3D s;
        qreal d;
        if (!intersectLine(a,b,c,u,v,s,d,0)) continue;
        if (d < dmin) { 
            dmin = d;
            pickPoint = s;
        }
    }

    std::cout << "pP = " << pickPoint.x() << " " << pickPoint.y() << " " << pickPoint.z() << std::endl;

    return (dmin < inf);
}

void MyGLWidget::wheelEvent(QWheelEvent* event) {
    QVector3D u,v;
    int x = event->x();
    int y = event->y();
    int delta = event->delta();

#ifdef MAC_QT5_BUG
    pickLine(2*x,2*y,u,v);
#else
    pickLine(x,y,u,v);
#endif
    QVector3D vu = v - u ;
    QVector3D cu = center - u;
    double l = QVector3D::dotProduct(vu,cu)/QVector3D::dotProduct(vu,vu);
    QVector3D p = u + vu*l;
    double factor = (delta < 0)? 1.2 : 1/1.2;
    zoom *= factor;
    center = p + (center-p)/factor;

    updateGL();
}

void MyGLWidget::mouseToTrackball(int x, int y, int W, int H, QVector3D &v) {
#ifdef MAC_QT5_BUG
    x*=2;
    y*=2;
#endif
    if (W > H) {
         v[0] = (2.0*x-W)/H;
         v[1] = 1.0-y*2.0/H;
    } else {
         v[0] = (2.0*x-W)/W;
         v[1] = (H-2.0*y)/W;
    }
    double d = v[0]*v[0]+v[1]*v[1];
    if (d > 1.0) {
         v[2] = 0.0;
         v /= sqrt(d);
    } else v[2] = sqrt(1.0-d*d);
}

QQuaternion MyGLWidget::trackball(const QVector3D& u, const QVector3D& v) {
    QVector3D uxv = QVector3D::crossProduct(u,v);
    qreal uv = QVector3D::dotProduct(u,v);
    QQuaternion ret(1+uv,uxv);
    ret.normalize();
    return ret;
}

void MyGLWidget::mousePressEvent(QMouseEvent *event) {
    button = event->button();
    oldX = event->x();
    oldY = event->y();

    if (event->modifiers() & Qt::ShiftModifier) {
        pointPicked = pick(oldX,oldY,pickPoint);
        std::cout << "pickPoint = [" << pickPoint.x() << "," << pickPoint.y() << "," << pickPoint.z() << "]" << std::endl;
    }

    updateGL();
}

void MyGLWidget::mouseReleaseEvent(QMouseEvent*) {}

void MyGLWidget::mouseMoveEvent(QMouseEvent* event) {
    int x = event->x();
    int y = event->y();

    if (button == Qt::LeftButton) {
        QVector3D p1,p2;

        mouseToTrackball(oldX,oldY,width,height,p1);
        mouseToTrackball(x,y,width,height,p2);

        QQuaternion q = trackball(p1,p2);
        QQuaternion qT = qNow.conjugate() * q.conjugate() * qNow;
        qNow = q * qNow;
        qNow.normalize();
        if (pointPicked)
            center = qT.rotatedVector(center - pickPoint) + pickPoint;

        // std::cout << "q = " << qNow.x() << " " << qNow.y() << " " << qNow.z() << " " << qNow.scalar() << std::endl;
    }

    if (button == Qt::RightButton) {
        QVector3D u0,u1,v0,v1;
        QVector3D vu,cu,p0,p1;
        double l;
        pickLine(oldX,oldY,u0,v0);

        vu = v0 - u0;
        cu = center - u0;
        l = QVector3D::dotProduct(vu,cu)/QVector3D::dotProduct(vu,vu);
        p0 = u0 + vu*l;

        pickLine(x,y,u1,v1);

        vu = v1 - u1;
        cu = center - u1;
        l = QVector3D::dotProduct(vu,cu)/QVector3D::dotProduct(vu,vu);
        p1 = u1 + vu*l;

        center -= p1 - p0;
    }

    oldX = x;
    oldY = y;

    updateGL();
}

void MyGLWidget::snapshotDualConturing(bool front_faces) {
    std::cout << "build edge data (position, normals) (" << (front_faces? "front faces)" : "back faces)") << std::endl;
    // initialize empty grid
    std::vector<GLint> emptyData(res*res*res);
    glEnable(GL_TEXTURE_3D);
    for (int i = 0; i < 3; ++i) {
        glBindTexture(GL_TEXTURE_3D, edges[i]);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAX_LEVEL, 0);
        glTexImage3D(GL_TEXTURE_3D, 0, GL_R32UI, res, res, res, 0, GL_RED_INTEGER, GL_UNSIGNED_INT, &emptyData[0]);
    }
    glBindTexture(GL_TEXTURE_3D, 0);
    glDisable(GL_CULL_FACE);

    // bind edge textures/images
    for (int i = 0; i < 3; ++i) {
        glActiveTexture(GL_TEXTURE0+i);
        glBindTexture(GL_TEXTURE_3D, edges[i]);
        glBindImageTexture(i, edges[i], 0, GL_TRUE, 0, GL_READ_WRITE, GL_R32UI);
    }
    snapshotDCProgram.bind();
    snapshotDCProgram.setUniformValue("edges[0]", 0);
    snapshotDCProgram.setUniformValue("edges[1]", 1);
    snapshotDCProgram.setUniformValue("edges[2]", 2);

    glEnable(GL_CULL_FACE);
    if (front_faces)
        glCullFace(GL_BACK);
    else
        glCullFace(GL_FRONT);
    snapshot(snapshotDCProgram);
    glDisable(GL_CULL_FACE);

    // unbind images
    for (int i = 0; i < 3; ++i) {
        glActiveTexture(GL_TEXTURE0+i);
        glBindImageTexture(i, 0, 0, GL_TRUE, 0, GL_READ_WRITE, GL_R32UI);
    }

}

void MyGLWidget::snapshot(QGLShaderProgram& program) {
    glDisable(GL_DEPTH_TEST);

    // set viewport to grid resolution
    glViewport(0,0,res,res);

    // set projection
    QMatrix4x4 P = QMatrix4x4();
    P.setToIdentity();
    P.ortho(-1.0,1.0,-1.0,1.0,-1.0+1.0/res,1.0-1.0/res);


    program.setUniformValue("uPMat", P);
    program.setUniformValue("res", res);

    //glBindBuffer(GL_ARRAY_BUFFER, vboSolidCubeId);
    //glBindBuffer(GL_ARRAY_BUFFER, vboSolidSphereId);
    // bind model VBO
    glBindBuffer(GL_ARRAY_BUFFER, vboTrianglesId);
    int vertexLocation = program.attributeLocation("a_position");
    program.enableAttributeArray(vertexLocation);
    glVertexAttribPointer(vertexLocation, 3, GL_FLOAT, GL_FALSE, 2*sizeof(QVector3D), 0);
    int normalLocation = program.attributeLocation("a_normal");
    program.enableAttributeArray(normalLocation);
    glVertexAttribPointer(normalLocation, 3, GL_FLOAT, GL_FALSE, 2*sizeof(QVector3D), (const void*) sizeof(QVector3D));

    vector<QMatrix4x4> tr;
    tr.push_back(QMatrix4x4());
    QMatrix4x4 T = QMatrix4x4();
    T.translate(0,0.05,0);
    tr.push_back(T);

    int n = 1; // the count of transformations to process
    int percent = 0;
    for (int j = 0; j < n; j++) {
        int newPercent = ((double)j/(double)n)*100.0;
        if (newPercent > percent) {
            percent = newPercent;
            std::cout << "rendering..." << percent << "%" << std::endl;
        }
        // render x,y,z view
        for (int i = 0; i < 3; i++) {
            QMatrix4x4 V = QMatrix4x4();

            V.scale(scaling, scaling, scaling); // scale model to fit in unit cube
            switch (i) {
                case 0:
                    V.rotate(90.0,0,1,0); // x-view
                    break;
                case 1:
                    V.rotate(-90.0,1,0,0); // y-view
                    break;
                case 2:
                    V.rotate(180.0,0,1,0); // z-view
                    break;
            }

            QMatrix4x4 M = QMatrix4x4();
            //M.rotate(32, 1, 0.5, 0);
            //M.scale(0.5,0.5,0.5);


            M.translate(-modelCenter);
            //M *= tr[j];
            program.setUniformValue("view", i);
            program.setUniformValue("uMVMat", V*M);
            program.setUniformValue("uNMat", M.normalMatrix());

            glDrawArrays(GL_TRIANGLES, 0, vboTrianglesSize);
            //glDrawArrays(GL_TRIANGLES, 0, vboSolidSphereSize);
            //glDrawArrays(GL_TRIANGLES, 0, vboSolidCubeSize);
        }
        glFinish();
        glFlush();
    }

    // reset
    resizeGL(width, height);
    glEnable(GL_DEPTH_TEST);
}

void MyGLWidget::copyEdgeData(std::vector<std::vector<GLuint>>& edgeData) {
    edgeData.resize(3);
    long dataSize = res*res*res;
    for (int i = 0; i < 3; ++i) {
        edgeData[i].resize(dataSize);
        glActiveTexture(GL_TEXTURE0+i);
        glGetTexImage(GL_TEXTURE_3D, 0, GL_RED_INTEGER, GL_UNSIGNED_INT, &edgeData[i][0]);
    }
}

void MyGLWidget::createAll() {
    //shift origin (-1,-1,-1) by half size of a grid cell
    origin = Vector3f(-1.0+1.0/res, -1.0+1.0/res, -1.0+1.0/res);

    snapshotDualConturing(true);
    std::cout << "copy edge data from graphic memory to RAM (front faces)" << std::endl;
    std::vector<std::vector<GLuint>> frontEdgeData;
    copyEdgeData(frontEdgeData);

    snapshotDualConturing(false);

    std::cout << "copy edge data from graphic memory to RAM (back faces)" << std::endl;
    std::vector<std::vector<GLuint>> backEdgeData;
    copyEdgeData(backEdgeData);

    meshGenerator.generate(frontEdgeData, backEdgeData, origin, 2.0f-2.0f/res);

    std::cout << "generate VBO for grid points inside the mesh" << std::endl;
    initGridVBO(meshGenerator.collector.grid);
    std::cout << "generate VBO for the octree cells" << std::endl;
    initCellVBO(meshGenerator.collector.cells);
    std::cout << "generate VBO for the edge cuts" << std::endl;
    initEdgeVBO(meshGenerator.collector.edgeCuts);
    std::cout << "generate VBO for the generated dual conturing vertices" << std::endl;
    initDCVerticesVBO(meshGenerator.collector.vertices);
    std::cout << "generate VBO of dual conturing mesh" << std::endl;
    initDCMeshVBO(meshGenerator.getVertices());

    // unbind textures
    for (int i = 0; i < 3; ++i) {
        glActiveTexture(GL_TEXTURE0+i);
        glBindTexture(GL_TEXTURE_3D, 0);
    }
    glDeleteTextures(3, &edges[0]);
    //writeToStlFile(meshGenerator.getVertices(), "Dual_Conturing.stl");

}



int main (int argc, char **argv) {
    QApplication app(argc, argv);

    if (!QGLFormat::hasOpenGL()) {
        qWarning ("This system has no OpenGL support. Exiting.");
        return 1;
    }

    w = new CGMainWindow(NULL);

    w->show();


    return app.exec();
}

