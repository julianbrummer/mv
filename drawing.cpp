#include "MV.h"

using namespace Eigen;
using namespace std;

template <typename VBO_DATA>
GLuint MyGLWidget::initVBO(const std::vector<VBO_DATA>& vboData) {
    GLuint id;
    glGenBuffers(1,&id);
    glBindBuffer(GL_ARRAY_BUFFER,id);
    glBufferData(GL_ARRAY_BUFFER,vboData.size()*sizeof(VBO_DATA),vboData.data(),GL_STATIC_DRAW);
    cout << vboData.size()*sizeof(VBO_DATA) << " Bytes" << endl;
    return id;
}

GLuint MyGLWidget::initIBO(const std::vector<uint>& iboData) {
    GLuint id;
    glGenBuffers(1,&id);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,id);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,iboData.size()*sizeof(uint),iboData.data(),GL_STATIC_DRAW);
    cout << iboData.size()*sizeof(uint) << " Bytes" << endl;
    return id;
}

void MyGLWidget::initVoxelVBO(const vector<bool>& voxel,const uint32_t size[3],const QVector3D& o,const qreal& s) {
    vector<QVector3D> vertexWithNormal;
    // cout << "begin initVoxelVBO" << endl;

    for(uint32_t z=0;z<size[2];z++)
        for(uint32_t y=0;y<size[1];y++)
            for(uint32_t x=0;x<size[0];x++) {
                uint32_t id = (z*size[1]+y)*size[0]+x;
                if (x < size[0]-1) {
                    if (!voxel[id] && voxel[id+1]) {
                        QVector3D a(o[0]+(x+1)*s,o[1]+y*s,o[2]+z*s);
                        QVector3D b(o[0]+(x+1)*s,o[1]+(y+1)*s,o[2]+z*s);
                        QVector3D c(o[0]+(x+1)*s,o[1]+(y+1)*s,o[2]+(z+1)*s);
                        QVector3D d(o[0]+(x+1)*s,o[1]+y*s,o[2]+(z+1)*s);
                        //QVector3D a(o[0]+(x+0)*s,o[1]+y*s,o[2]+z*s);
                        //QVector3D b(o[0]+(x+0)*s,o[1]+(y+1)*s,o[2]+z*s);
                        //QVector3D c(o[0]+(x+0)*s,o[1]+(y+1)*s,o[2]+(z+1)*s);
                        //QVector3D d(o[0]+(x+0)*s,o[1]+y*s,o[2]+(z+1)*s);
                        QVector3D n(-1,0,0);
                        vertexWithNormal.push_back(a);
                        vertexWithNormal.push_back(n);
                        vertexWithNormal.push_back(d);
                        vertexWithNormal.push_back(n);
                        vertexWithNormal.push_back(c);
                        vertexWithNormal.push_back(n);
                        vertexWithNormal.push_back(c);
                        vertexWithNormal.push_back(n);
                        vertexWithNormal.push_back(b);
                        vertexWithNormal.push_back(n);
                        vertexWithNormal.push_back(a);
                        vertexWithNormal.push_back(n);
                    }
                }

                if (x > 0) {
                    if (!voxel[id] && voxel[id-1]) {
                        QVector3D a(o[0]+x*s,o[1]+y*s,o[2]+z*s);
                        QVector3D b(o[0]+x*s,o[1]+(y+1)*s,o[2]+z*s);
                        QVector3D c(o[0]+x*s,o[1]+(y+1)*s,o[2]+(z+1)*s);
                        QVector3D d(o[0]+x*s,o[1]+y*s,o[2]+(z+1)*s);
                        QVector3D n(1,0,0);
                        vertexWithNormal.push_back(a);
                        vertexWithNormal.push_back(n);
                        vertexWithNormal.push_back(b);
                        vertexWithNormal.push_back(n);
                        vertexWithNormal.push_back(c);
                        vertexWithNormal.push_back(n);
                        vertexWithNormal.push_back(c);
                        vertexWithNormal.push_back(n);
                        vertexWithNormal.push_back(d);
                        vertexWithNormal.push_back(n);
                        vertexWithNormal.push_back(a);
                        vertexWithNormal.push_back(n);
                    }
                }

                if (y < size[1]-1) {
                    if (!voxel[id] && voxel[id+size[0]]) {
                        QVector3D a(o[0]+x*s,o[1]+(y+1)*s,o[2]+z*s);
                        QVector3D b(o[0]+(x+1)*s,o[1]+(y+1)*s,o[2]+z*s);
                        QVector3D c(o[0]+(x+1)*s,o[1]+(y+1)*s,o[2]+(z+1)*s);
                        QVector3D d(o[0]+x*s,o[1]+(y+1)*s,o[2]+(z+1)*s);
                        QVector3D n(0,-1,0);
                        vertexWithNormal.push_back(a);
                        vertexWithNormal.push_back(n);
                        vertexWithNormal.push_back(b);
                        vertexWithNormal.push_back(n);
                        vertexWithNormal.push_back(c);
                        vertexWithNormal.push_back(n);
                        vertexWithNormal.push_back(c);
                        vertexWithNormal.push_back(n);
                        vertexWithNormal.push_back(d);
                        vertexWithNormal.push_back(n);
                        vertexWithNormal.push_back(a);
                        vertexWithNormal.push_back(n);
                    }
                }

                if (y > 0) {
                    if (!voxel[id] && voxel[id-size[0]]) {
                        QVector3D a(o[0]+x*s,o[1]+y*s,o[2]+z*s);
                        QVector3D b(o[0]+(x+1)*s,o[1]+y*s,o[2]+z*s);
                        QVector3D c(o[0]+(x+1)*s,o[1]+y*s,o[2]+(z+1)*s);
                        QVector3D d(o[0]+x*s,o[1]+y*s,o[2]+(z+1)*s);
                        QVector3D n(0,1,0);
                        vertexWithNormal.push_back(a);
                        vertexWithNormal.push_back(n);
                        vertexWithNormal.push_back(d);
                        vertexWithNormal.push_back(n);
                        vertexWithNormal.push_back(c);
                        vertexWithNormal.push_back(n);
                        vertexWithNormal.push_back(c);
                        vertexWithNormal.push_back(n);
                        vertexWithNormal.push_back(b);
                        vertexWithNormal.push_back(n);
                        vertexWithNormal.push_back(a);
                        vertexWithNormal.push_back(n);
                    }
                }

                if (z < size[2]-1) {
                    if (!voxel[id] && voxel[id+size[0]*size[1]]) {
                        QVector3D a(o[0]+x*s,o[1]+y*s,o[2]+(z+1)*s);
                        QVector3D b(o[0]+(x+1)*s,o[1]+y*s,o[2]+(z+1)*s);
                        QVector3D c(o[0]+(x+1)*s,o[1]+(y+1)*s,o[2]+(z+1)*s);
                        QVector3D d(o[0]+x*s,o[1]+(y+1)*s,o[2]+(z+1)*s);
                        QVector3D n(0,0,-1);
                        vertexWithNormal.push_back(a);
                        vertexWithNormal.push_back(n);
                        vertexWithNormal.push_back(d);
                        vertexWithNormal.push_back(n);
                        vertexWithNormal.push_back(c);
                        vertexWithNormal.push_back(n);
                        vertexWithNormal.push_back(c);
                        vertexWithNormal.push_back(n);
                        vertexWithNormal.push_back(b);
                        vertexWithNormal.push_back(n);
                        vertexWithNormal.push_back(a);
                        vertexWithNormal.push_back(n);
                    }
                }

                if (z > 0) {
                    if (!voxel[id] && voxel[id-size[0]*size[1]]) {
                        QVector3D a(o[0]+x*s,o[1]+y*s,o[2]+z*s);
                        QVector3D b(o[0]+(x+1)*s,o[1]+y*s,o[2]+z*s);
                        QVector3D c(o[0]+(x+1)*s,o[1]+(y+1)*s,o[2]+z*s);
                        QVector3D d(o[0]+x*s,o[1]+(y+1)*s,o[2]+z*s);
                        QVector3D n(0,0,1);
                        vertexWithNormal.push_back(a);
                        vertexWithNormal.push_back(n);
                        vertexWithNormal.push_back(b);
                        vertexWithNormal.push_back(n);
                        vertexWithNormal.push_back(c);
                        vertexWithNormal.push_back(n);
                        vertexWithNormal.push_back(c);
                        vertexWithNormal.push_back(n);
                        vertexWithNormal.push_back(d);
                        vertexWithNormal.push_back(n);
                        vertexWithNormal.push_back(a);
                        vertexWithNormal.push_back(n);
                    }
                }
            }

    vboVoxelId = initVBO(vertexWithNormal);
    vboVoxelSize = static_cast<int>(vertexWithNormal.size()/2);
}

void MyGLWidget::initCellVBO(const vector<Vertex>& vertices) {
    vboCellId = initVBO(vertices);
    vboCellSize = vertices.size();
}

void MyGLWidget::initEdgeVBO(const vector<Vertex>& vertices) {
    vboEdgeId = initVBO(vertices);
    vboEdgeSize = vertices.size();
}

void MyGLWidget::initGridVBO(const vector<Vertex>& vertices) {
    vboGridId = initVBO(vertices);
    vboGridSize = vertices.size();
}

void MyGLWidget::initDCVerticesVBO(const vector<Vertex>& vertices) {
    vboDCVerticesId = initVBO(vertices);
    vboDCVerticesSize = vertices.size();
}

void MyGLWidget::initDCMeshVBO(const vector<Vertex>& vertices) {
    vboDCMeshId = initVBO(vertices);
    vboDCMeshSize = vertices.size();
}

void MyGLWidget::initDCMeshIBO(const vector<uint>& indices) {
    iboDCMeshId = initIBO(indices);
    iboDCMeshSize = indices.size();
}


/*void MyGLWidget::edgePoints(const vector<bool>& points, const vector<vector<GLuint>>& frontEdges, const std::vector<std::vector<GLuint> > &backEdges, const uint32_t size,
                            const Vector3d& o, const qreal& s, std::vector<QVector3D>& vertexWithNormal) {
    for (int view = 0; view < 3; view++)
        for(uint32_t z = 0; z < size; z++) {
            for(uint32_t y = 0; y < size; y++)
                for(uint32_t x = 0; x < size; x++) {
                    Edge e = edge(points, frontEdges, x, y, z, view, size, o, s);
                    if (e.hasCut) {
                        vertexWithNormal.push_back(e.cutPos());
                        vertexWithNormal.push_back(e.normal);
                    }
                    e = edge(points, backEdges, x, y, z, view, size, o, s);
                    if (e.hasCut) {
                        vertexWithNormal.push_back(e.cutPos());
                        vertexWithNormal.push_back(e.normal);
                    }
                }
        }

}*/

void MyGLWidget::initSolidCubeVBO() {
    vector<QVector3D> vertexWithNormal;
    GLuint id;
    glGenBuffers(1,&id);

    vertexWithNormal.push_back(QVector3D(-1,-1,-1));
    vertexWithNormal.push_back(QVector3D( 0,-1, 0));
    vertexWithNormal.push_back(QVector3D( 1,-1,-1));
    vertexWithNormal.push_back(QVector3D( 0,-1, 0));
    vertexWithNormal.push_back(QVector3D( 1,-1, 1));
    vertexWithNormal.push_back(QVector3D( 0,-1, 0));
    vertexWithNormal.push_back(QVector3D( 1,-1, 1));
    vertexWithNormal.push_back(QVector3D( 0,-1, 0));
    vertexWithNormal.push_back(QVector3D(-1,-1, 1));
    vertexWithNormal.push_back(QVector3D( 0,-1, 0));
    vertexWithNormal.push_back(QVector3D(-1,-1,-1));
    vertexWithNormal.push_back(QVector3D( 0,-1, 0));

    vertexWithNormal.push_back(QVector3D(-1, 1,-1));
    vertexWithNormal.push_back(QVector3D( 0, 1, 0));
    vertexWithNormal.push_back(QVector3D(-1, 1, 1));
    vertexWithNormal.push_back(QVector3D( 0, 1, 0));
    vertexWithNormal.push_back(QVector3D( 1, 1, 1));
    vertexWithNormal.push_back(QVector3D( 0, 1, 0));
    vertexWithNormal.push_back(QVector3D( 1, 1, 1));
    vertexWithNormal.push_back(QVector3D( 0, 1, 0));
    vertexWithNormal.push_back(QVector3D( 1, 1,-1));
    vertexWithNormal.push_back(QVector3D( 0, 1, 0));
    vertexWithNormal.push_back(QVector3D(-1, 1,-1));
    vertexWithNormal.push_back(QVector3D( 0, 1, 0));

    vertexWithNormal.push_back(QVector3D(-1, 1, 1));
    vertexWithNormal.push_back(QVector3D(-1, 0, 0));
    vertexWithNormal.push_back(QVector3D(-1, 1,-1));
    vertexWithNormal.push_back(QVector3D(-1, 0, 0));
    vertexWithNormal.push_back(QVector3D(-1,-1,-1));
    vertexWithNormal.push_back(QVector3D(-1, 0, 0));
    vertexWithNormal.push_back(QVector3D(-1,-1,-1));
    vertexWithNormal.push_back(QVector3D(-1, 0, 0));
    vertexWithNormal.push_back(QVector3D(-1,-1, 1));
    vertexWithNormal.push_back(QVector3D(-1, 0, 0));
    vertexWithNormal.push_back(QVector3D(-1, 1, 1));
    vertexWithNormal.push_back(QVector3D(-1, 0, 0));

    vertexWithNormal.push_back(QVector3D( 1, 1, 1));
    vertexWithNormal.push_back(QVector3D( 1, 0, 0));
    vertexWithNormal.push_back(QVector3D( 1,-1, 1));
    vertexWithNormal.push_back(QVector3D( 1, 0, 0));
    vertexWithNormal.push_back(QVector3D( 1,-1,-1));
    vertexWithNormal.push_back(QVector3D( 1, 0, 0));
    vertexWithNormal.push_back(QVector3D( 1,-1,-1));
    vertexWithNormal.push_back(QVector3D( 1, 0, 0));
    vertexWithNormal.push_back(QVector3D( 1, 1,-1));
    vertexWithNormal.push_back(QVector3D( 1, 0, 0));
    vertexWithNormal.push_back(QVector3D( 1, 1, 1));
    vertexWithNormal.push_back(QVector3D( 1, 0, 0));

    vertexWithNormal.push_back(QVector3D( 1,-1,-1));
    vertexWithNormal.push_back(QVector3D( 0, 0,-1));
    vertexWithNormal.push_back(QVector3D(-1,-1,-1));
    vertexWithNormal.push_back(QVector3D( 0, 0,-1));
    vertexWithNormal.push_back(QVector3D(-1, 1,-1));
    vertexWithNormal.push_back(QVector3D( 0, 0,-1));
    vertexWithNormal.push_back(QVector3D(-1, 1,-1));
    vertexWithNormal.push_back(QVector3D( 0, 0,-1));
    vertexWithNormal.push_back(QVector3D( 1, 1,-1));
    vertexWithNormal.push_back(QVector3D( 0, 0,-1));
    vertexWithNormal.push_back(QVector3D( 1,-1,-1));
    vertexWithNormal.push_back(QVector3D( 0, 0,-1));

    vertexWithNormal.push_back(QVector3D(-1, 1, 1));
    vertexWithNormal.push_back(QVector3D( 0, 0, 1));
    vertexWithNormal.push_back(QVector3D(-1,-1, 1));
    vertexWithNormal.push_back(QVector3D( 0, 0, 1));
    vertexWithNormal.push_back(QVector3D( 1,-1, 1));
    vertexWithNormal.push_back(QVector3D( 0, 0, 1));
    vertexWithNormal.push_back(QVector3D( 1,-1, 1));
    vertexWithNormal.push_back(QVector3D( 0, 0, 1));
    vertexWithNormal.push_back(QVector3D( 1, 1, 1));
    vertexWithNormal.push_back(QVector3D( 0, 0, 1));
    vertexWithNormal.push_back(QVector3D(-1, 1, 1));
    vertexWithNormal.push_back(QVector3D( 0, 0, 1));

    glBindBuffer(GL_ARRAY_BUFFER,id);
    glBufferData(GL_ARRAY_BUFFER,vertexWithNormal.size()*sizeof(QVector3D),vertexWithNormal.data(),GL_STATIC_DRAW);

    vboSolidCubeId = id;
    vboSolidCubeSize = 3*12;
}

void MyGLWidget::refineSolidSphere(const vector<QVector3D>& sphere,vector<QVector3D>& refined) {
    for(size_t i=0;i<sphere.size()/3;i++) {
        const QVector3D& a = sphere[3*i+0];
        const QVector3D& b = sphere[3*i+1];
        const QVector3D& c = sphere[3*i+2];
        
        QVector3D ab = a+b;
        QVector3D bc = b+c;
        QVector3D ca = c+a;
        
        ab.normalize();
        bc.normalize();
        ca.normalize();
        
        refined.push_back(a);
        refined.push_back(ab);
        refined.push_back(ca);
        
        refined.push_back(ab);
        refined.push_back(b);
        refined.push_back(bc);
        
        refined.push_back(bc);
        refined.push_back(c);
        refined.push_back(ca);
        
        refined.push_back(ab);
        refined.push_back(bc);
        refined.push_back(ca);
    }
}

void MyGLWidget::initSolidSphereVBO() {
    vector<QVector3D> ico;
    qreal gr = 0.5*(1.0+sqrt(5.0));

    ico.push_back( QVector3D(gr,1.0,0.0));
    ico.push_back( QVector3D(1.0,0.0,gr));
    ico.push_back( QVector3D(gr,-1.0,0.0)); 
    
    ico.push_back( QVector3D(gr,1.0,0.0));
    ico.push_back( QVector3D(gr,-1.0,0.0));
    ico.push_back( QVector3D(1.0,0.0,-gr));
    
    ico.push_back( QVector3D(gr,1.0,0.0));
    ico.push_back( QVector3D(0.0,gr,-1.0));
    ico.push_back( QVector3D(0.0,gr,1.0));
    
    ico.push_back( QVector3D(gr,1.0,0.0));
    ico.push_back( QVector3D(0.0,gr,1.0));
    ico.push_back( QVector3D(1.0,0.0,gr));
    
    ico.push_back( QVector3D(gr,1.0,0.0));
    ico.push_back( QVector3D(1.0,0.0,-gr));
    ico.push_back( QVector3D(0.0,gr,-1.0));
    
    ico.push_back( QVector3D(-gr,-1.0,0.0));
    ico.push_back( QVector3D(-1.0,0.0,gr));
    ico.push_back( QVector3D(-gr,1.0,0.0));
    
    ico.push_back( QVector3D(-gr,-1.0,0.0));
    ico.push_back( QVector3D(-gr,1.0,0.0));
    ico.push_back( QVector3D(-1.0,0.0,-gr));
    
    ico.push_back( QVector3D(-gr,-1.0,0.0));
    ico.push_back( QVector3D(0.0,-gr,-1.0));
    ico.push_back( QVector3D(0.0,-gr,1.0));
    
    ico.push_back( QVector3D(-gr,-1.0,0.0));
    ico.push_back( QVector3D(0.0,-gr,1.0));
    ico.push_back( QVector3D(-1.0,0.0,gr));
    
    ico.push_back( QVector3D(-gr,-1.0,0.0));
    ico.push_back( QVector3D(-1.0,0.0,-gr));
    ico.push_back( QVector3D(0.0,-gr,-1.0));
    
    ico.push_back( QVector3D(1.0,0.0,gr));
    ico.push_back( QVector3D(-1.0,0.0,gr));
    ico.push_back( QVector3D(0.0,-gr,1.0));
    
    ico.push_back( QVector3D(1.0,0.0,gr));
    ico.push_back( QVector3D(0.0,gr,1.0));
    ico.push_back( QVector3D(-1.0,0.0,gr));
    
    ico.push_back( QVector3D(0.0,gr,1.0));
    ico.push_back( QVector3D(-gr,1.0,0.0));
    ico.push_back( QVector3D(-1.0,0.0,gr));
    
    ico.push_back( QVector3D(0.0,gr,1.0));
    ico.push_back( QVector3D(0.0,gr,-1.0));
    ico.push_back( QVector3D(-gr,1.0,0.0));
    
    ico.push_back( QVector3D(0.0,gr,-1.0));
    ico.push_back( QVector3D(-1.0,0.0,-gr));
    ico.push_back( QVector3D(-gr,1.0,0.0));
    
    ico.push_back( QVector3D(-1.0,0.0,-gr));
    ico.push_back( QVector3D(0.0,gr,-1.0));
    ico.push_back( QVector3D(1.0,0.0,-gr));
    
    ico.push_back( QVector3D(-1.0,0.0,-gr));
    ico.push_back( QVector3D(1.0,0.0,-gr));
    ico.push_back( QVector3D(0.0,-gr,-1.0));
    
    ico.push_back( QVector3D(0.0,-gr,-1.0));
    ico.push_back( QVector3D(1.0,0.0,-gr));
    ico.push_back( QVector3D(gr,-1.0,0.0));
    
    ico.push_back( QVector3D(0.0,-gr,-1.0));
    ico.push_back( QVector3D(gr,-1.0,0.0));
    ico.push_back( QVector3D(0.0,-gr,1.0));
    
    ico.push_back( QVector3D(0.0,-gr,1.0));
    ico.push_back( QVector3D(gr,-1.0,0.0));
    ico.push_back( QVector3D(1.0,0.0,gr));

    for(size_t i=0;i<ico.size();i++) ico[i].normalize();

    for(int i=0;i<3;i++) {
        vector<QVector3D> ico_refined;
        refineSolidSphere(ico,ico_refined);
        ico = ico_refined;
    }

    vector<QVector3D> vertexWithNormal;
    GLuint id;
    glGenBuffers(1,&id);

    for(size_t i=0;i<ico.size();i++) {
	vertexWithNormal.push_back(ico[i]);
	vertexWithNormal.push_back(ico[i]);
    }

    glBindBuffer(GL_ARRAY_BUFFER,id);
    glBufferData(GL_ARRAY_BUFFER,vertexWithNormal.size()*sizeof(QVector3D),vertexWithNormal.data(),GL_STATIC_DRAW);

    vboSolidSphereId = id;
    vboSolidSphereSize = static_cast<int>(ico.size());
}

void MyGLWidget::initSolidCylinderVBO() {
    vector<QVector3D> vertexWithNormal;
    const int n = 64;
    vector<QVector3D> circle(n);

    circle[0]     = QVector3D( 1.0, 0.0, 0.0);
    circle[n/4]   = QVector3D( 0.0, 1.0, 0.0);
    circle[n/2]   = QVector3D(-1.0, 0.0, 0.0);
    circle[3*n/4] = QVector3D( 0.0,-1.0, 0.0);

    for(int r=n/4;r>1;r/=2)
       for(int i=0;i<n;i+=r) {
          circle[i+r/2] = circle[i]+circle[(i+r)%n];
          circle[i+r/2].normalize(); 
       }

    QVector3D ez(0.0,0.0,1.0);

    for(int i=0;i<n;i++) {
       vertexWithNormal.push_back(circle[i]);
       vertexWithNormal.push_back(circle[i]);
       vertexWithNormal.push_back(circle[(i+1)%n]);
       vertexWithNormal.push_back(circle[(i+1)%n]);
       vertexWithNormal.push_back(circle[i]+ez);
       vertexWithNormal.push_back(circle[i]);

       vertexWithNormal.push_back(circle[(i+1)%n]+ez);
       vertexWithNormal.push_back(circle[(i+1)%n]);
       vertexWithNormal.push_back(circle[i]+ez);
       vertexWithNormal.push_back(circle[i]);
       vertexWithNormal.push_back(circle[(i+1)%n]);
       vertexWithNormal.push_back(circle[(i+1)%n]);
    }

    GLuint id;
    glGenBuffers(1,&id);

    glBindBuffer(GL_ARRAY_BUFFER,id);
    glBufferData(GL_ARRAY_BUFFER,vertexWithNormal.size()*sizeof(QVector3D),vertexWithNormal.data(),GL_STATIC_DRAW);

    vboSolidCylinderId = id;
    vboSolidCylinderSize = static_cast<int>(vertexWithNormal.size()/2);
}

void MyGLWidget::initTrianglesVBO(const vector<QVector3D>& triangles) {
    vector<QVector3D> vertexWithNormal;
	GLuint id;
	glGenBuffers(1,&id);

	for(size_t i=0;i<triangles.size();i+=3) {
		const QVector3D& a = triangles[i+0];
		const QVector3D& b = triangles[i+1];
		const QVector3D& c = triangles[i+2];
		QVector3D n = QVector3D::crossProduct(b-a,c-a);
		n.normalize();
		vertexWithNormal.push_back(a);
		vertexWithNormal.push_back(n);

		vertexWithNormal.push_back(b);
		vertexWithNormal.push_back(n);

		vertexWithNormal.push_back(c);
		vertexWithNormal.push_back(n);
	}

	glBindBuffer(GL_ARRAY_BUFFER,id);
	glBufferData(GL_ARRAY_BUFFER,vertexWithNormal.size()*sizeof(QVector3D),vertexWithNormal.data(),GL_STATIC_DRAW);

    vboTrianglesId = id;
    vboTrianglesSize = static_cast<int>(triangles.size());
}

void MyGLWidget::initAxisVBO(float size) {
    std::cout << "generate Axis VBO" << std::endl;
    vector<Vector3f> vertexWithColor;
    vertexWithColor.push_back(Vector3f(0,0,0));
    vertexWithColor.push_back(Vector3f(1,0,0));
    vertexWithColor.push_back(Vector3f(size,0,0));
    vertexWithColor.push_back(Vector3f(1,0,0));

    vertexWithColor.push_back(Vector3f(0,0,0));
    vertexWithColor.push_back(Vector3f(0,1,0));
    vertexWithColor.push_back(Vector3f(0,size,0));
    vertexWithColor.push_back(Vector3f(0,1,0));

    vertexWithColor.push_back(Vector3f(0,0,0));
    vertexWithColor.push_back(Vector3f(0,0,1));
    vertexWithColor.push_back(Vector3f(0,0,size));
    vertexWithColor.push_back(Vector3f(0,0,1));

    vboAxisId = initVBO(vertexWithColor);
    vboAxisSize = static_cast<int>(vertexWithColor.size()/2);

}

QMatrix4x4 orthonormalSystem(QVector3D e3) {
	qreal u[3],v[3];
        e3.normalize();
	v[0] = e3.x();
	v[1] = e3.y();
	v[2] = e3.z();
	int imax = 0;
	if (fabs(v[1]) > fabs(v[imax])) imax = 1;
	if (fabs(v[2]) > fabs(v[imax])) imax = 2;
	u[imax] = v[(imax+1)%3];
	u[(imax+1)%3] = -v[imax];
	u[(imax+2)%3] = 0.0;
	QVector3D e2(u[0],u[1],u[2]);
	e2.normalize();
	QVector3D e1 = QVector3D::crossProduct(e2,e3);
	e1.normalize();

	return QMatrix4x4(e1.x(),e2.x(),e3.x(),0.0,
	                  e1.y(),e2.y(),e3.y(),0.0,
	                  e1.z(),e2.z(),e3.z(),0.0,
	                  0.0,   0.0,   0.0,   1.0);
}

void MyGLWidget::drawSolidCylinder(const QVector3D& a, const QVector3D& b, qreal radius) {
	QMatrix4x4 M(modelView);
        M.translate(a);
	M *= orthonormalSystem(b-a);
        M.scale(radius,radius,(b-a).length());

	program.setUniformValue("uMVMat", M);
	program.setUniformValue("uNMat", M.normalMatrix());

	glBindBuffer(GL_ARRAY_BUFFER, vboSolidCylinderId);
	int vertexLocation = program.attributeLocation("a_position");
	program.enableAttributeArray(vertexLocation);
	glVertexAttribPointer(vertexLocation, 3, GL_FLOAT, GL_FALSE, 2*sizeof(QVector3D), 0);
	int normalLocation = program.attributeLocation("a_normal");
	program.enableAttributeArray(normalLocation);
	glVertexAttribPointer(normalLocation, 3, GL_FLOAT, GL_FALSE, 2*sizeof(QVector3D), (const void*) sizeof(QVector3D));

	glDrawArrays(GL_TRIANGLES,0,vboSolidCylinderSize);
}

void MyGLWidget::drawSolidSphere(const QVector3D& c, qreal r) {
	QMatrix4x4 M(modelView);
	M.translate(c); 
	M.scale(r);

	program.setUniformValue("uMVMat", M);
	program.setUniformValue("uNMat", M.normalMatrix());

	glBindBuffer(GL_ARRAY_BUFFER, vboSolidSphereId);
	int vertexLocation = program.attributeLocation("a_position");
	program.enableAttributeArray(vertexLocation);
	glVertexAttribPointer(vertexLocation, 3, GL_FLOAT, GL_FALSE, 2*sizeof(QVector3D), 0);
	int normalLocation = program.attributeLocation("a_normal");
	program.enableAttributeArray(normalLocation);
	glVertexAttribPointer(normalLocation, 3, GL_FLOAT, GL_FALSE, 2*sizeof(QVector3D), (const void*) sizeof(QVector3D));

	glDrawArrays(GL_TRIANGLES,0,vboSolidSphereSize);
}

void MyGLWidget::drawVBO(int id, int size, QGLShaderProgram& program, GLenum mode, bool normal, bool color) {
    int offset = 0;
    int stride = sizeof(Vector3f) + (normal? sizeof(Vector3f):0) + (color? sizeof(Vector3f):0);
    glBindBuffer(GL_ARRAY_BUFFER, id);
    int vertexLocation = program.attributeLocation("a_position");
    if (vertexLocation >= 0) {
        program.enableAttributeArray(vertexLocation);
        glVertexAttribPointer(vertexLocation, 3, GL_FLOAT, GL_FALSE, stride, (const void*) offset);
        offset += sizeof(Vector3f);
    }
    if (normal) {
        int normalLocation = program.attributeLocation("a_normal");
        if (normalLocation >= 0) {
            program.enableAttributeArray(normalLocation);
            glVertexAttribPointer(normalLocation, 3, GL_FLOAT, GL_FALSE, stride, (const void*) offset);
        }
        offset += sizeof(Vector3f);
    }
    if (color) {
        int colorLocation = program.attributeLocation("a_color");
        if (colorLocation >= 0) {
            program.enableAttributeArray(colorLocation);
            glVertexAttribPointer(colorLocation, 3, GL_FLOAT, GL_FALSE, stride, (const void*) offset);
        }
        offset += sizeof(Vector3f);
    }
    glDrawArrays(mode,0,size);
}

