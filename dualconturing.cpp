#include <dualconturing.h>
#include <iostream>
#include <Eigen/Dense>
#include <set>
#include <queue>

typedef uint8_t byte;

using namespace Eigen;
using namespace std;

//==================================
//Edge
//==================================

Edge::Edge() : Edge(-1) {}

Edge::Edge(int orientation): front_normal(nullptr), back_normal(nullptr), front_d(0.0f), back_d(0.0f),
                                                     orientation(orientation), hasFrontCut(false), hasBackCut(false) {
    p[0] = false;
    p[1] = false;
}

Edge::~Edge() {
    delete front_normal;
    delete back_normal;
    front_normal = nullptr;
    back_normal = nullptr;
}


//==================================
//Vertex
//==================================

Vertex::Vertex(const Vector3f& position, const Vector3f &color): Vertex(position, Vector3f(0,0,0), color) {}

Vertex::Vertex(const Vector3f& position, const Vector3f& normal, const Vector3f& color):
    position(position), normal(normal), color(color) {}

//==================================
//QEF
//==================================

QEF::QEF() {
    for (int i = 0; i < 6; ++i) {
        a[i] = 0.0;
    }
    b.setZero(3);
    c = 0.0;
    m.setZero(3);
    dimension = SolutionSpace::UNDEFINED;
}

void QEF::add(const Vector3f& normal, const Vector3f& point) {
    //E(x) = (n^T(x - p))^2
    //E(x) = (n^Tx - n^Tp)^2 = x^T(nn^T)x + 2d*n^Tx + d^2
    //grad(E(x)) = 2Ax + 2b = 0
    // Ax = -b
    Vector3d n = normal.cast<double>();
    Vector3d p = point.cast<double>();
    double d = - n.transpose() * p; // n^T p
    Matrix3d A = n * n.transpose(); //nn^T
    a[0] += A(0,0);
    a[1] += A(0,1);
    a[2] += A(0,2);
    a[3] += A(1,1);
    a[4] += A(1,2);
    a[5] += A(2,2);
    b += d*n;
    c += d*d;
}

void QEF::add(const QEF& qef) {
    for (int i = 0; i < 6; ++i) {
        a[i] += qef.a[i];
    }
    b += qef.b;
    c += qef.c;
}

SolutionSpace QEF::solve(const Vector3f &m, Vector3f& c) {
    // solve for minimizer x = c+m of Ax = -b with minimal distance to m
    // -b = A(c+m) = Ac + Am
    // -Am - b = Ac
    // c = A^+ (-Am - b)
    Matrix3d A;
    A << a[0], a[1], a[2],
         a[1], a[3], a[4],
         a[2], a[4], a[5];
    JacobiSVD<Matrix3Xd> svd(A, ComputeThinU | ComputeThinV); // U D V^T
    svd.setThreshold(truncation/svd.singularValues()[0]); // set singular values <= truncation to zero
                            // to avoid high values in diagonal matrix D^+

    // solve for c
    c = svd.solve(-A*m.cast<double>()-b).cast<float>();
    dimension = static_cast<SolutionSpace>(svd.rank());
    return dimension;
}

SolutionSpace QEF::solve(Vector3f& c) {
    return solve(m, c);
}

double QEF::evaluate(const Vector3f& v) const {
    Matrix3d A;
    Vector3d x = v.cast<double>();
    A << a[0], a[1], a[2],
         a[1], a[3], a[4],
         a[2], a[4], a[5];
    double dA = x.transpose()*A*x;
    double db = 2*b.transpose()*x;
    return dA + db + c;
}


//==================================
//Cell
//==================================

Cell::Cell(const int level) : v(nullptr), v_opt(nullptr), vertIndex(-1), level(level),
                              qef(nullptr) {
    for (int i = 0; i < 8; ++i)
        p[i] = false;
}

bool Cell::isInside() const {
    for (int i = 0; i < 8; ++i) {
        if (!p[i])
           return false;
    }
    return true;
}

bool Cell::inCell(Vector3f& pos, const Vector3f& cellOrigin, const float size) const {
    if (pos[0] <= size+cellOrigin[0]+eps && pos[1] <= size+cellOrigin[1]+eps && pos[2] <= size+cellOrigin[2]+eps &&
           pos[0] >= cellOrigin[0]-eps && pos[1] >= cellOrigin[1]-eps && pos[2] >= cellOrigin[2]-eps) {
        // clamp to cell
        pos[0] = min(max(pos[0], cellOrigin[0]), size+cellOrigin[0]);
        pos[1] = min(max(pos[1], cellOrigin[1]), size+cellOrigin[1]);
        pos[2] = min(max(pos[2], cellOrigin[2]), size+cellOrigin[2]);
        return true;
    }
    return false;
}

SolutionSpace Cell::solveQEF(const Vector3f& m, Vector3f& v) const {
    dc::VertexGenerator& g = dc::VertexGenerator::getInstance();

    SolutionSpace rank = qef->solve(m, v); // relative to m
    v += m; // relative to grid origin
    return rank;
}

SolutionSpace Cell::solveQEF(Vector3f& v) const {
    return solveQEF(qef->m, v);
}

void Cell::projectToSphere(const Vector3f& center, const float r, Vector3f& v) const {
    Vector3f dir = v - center;
    dir.normalize();
    v = center + r*dir;
}


bool Cell::intersect(const Vector3f& cellOrigin, const float size, Vector3f& v0, const Vector3f& v1) const {
    Vector3f intersection;
    bool intersect = false;

    Vector3f dir = v1 - v0;
    for (int i = 0; i < 6; ++i) {
        uint j = plane_orientation[i];
        if (dir[j] != 0.0)  {
            float s = (cellOrigin[j] + plane_shift[i]*size - v0[j])/dir[j];
            Vector3f v = v0 + s*dir;
            if (inCell(v, cellOrigin, size)) { // intersect?
                if (intersect) { // already intersected once?
                    v0 = (intersection+v)*0.5f; // center of intersections
                    return true;
                } else { // first intersection
                    intersection = v;
                    intersect = true;
                }
            }
        }
    }
    return false;
}

float Cell::intersect(const Vector3f& n, const Vector3f& p, const Vector3f& c, const uint orientation) const {
    //edge e: c + x where shift c_i == 0 or cellSize and x_i = 0 for all i != orientation
    //plane E: n^T(v-p) = 0
    // x_orientation = [n^T(p-c)]/n_orientation

    if (n[orientation] == 0.0) // plane parallel to edge
        return -1.0f;
    float x = n.transpose() * (p-c);
    return x/n[orientation];
}



bool Cell::intersect(const Vector3f& cellOrigin, const float size, Vector3f& v0, const Vector3f& v1, const Vector3f& v2) const {
    vector<Vector3f> isecs; // intersections
    isecs.reserve(4);
    Vector3f n = (v1-v0).cross(v2-v0);
    for (int i = 0; i < 12; ++i) {
        Vector3f c = cellOrigin + edge_origin[i].cast<float>()*size; // edge origin
        Vector3f v = c; // intersection
        v[edge_orientation[i]] += intersect(n, v0, c, edge_orientation[i]);
        if (inCell(v, cellOrigin, size)) {
            isecs.push_back(v);
            if (isecs.size() == 4) {
                break;
            }
        }
    }

    if (isecs.size() == 0)
        return false;

    // center of intersections
    v0 = Vector3f(0,0,0);
    for (uint i = 0; i < isecs.size(); ++i) {
        v0 += isecs[i];
    }
    v0 /= isecs.size();

    return true;
}

void Cell::generateVertex(const Vector3f& cellOrigin, const float size) {
    dc::VertexGenerator& g = dc::VertexGenerator::getInstance();
    bool projected = false;
    Vector3f col = Color::WHITE;
    float r = size/2.0f;
    Vector3f center = cellOrigin + Vector3f(r, r, r);
    Vector3f v0(0,0,0);
    Vector3f opt(0,0,0); // the optimal position
    switch (solveQEF(v0)) {
        case POINT_SPACE:
            if (!inCell(v0, cellOrigin, size)) {
                opt = v0;
                projectToSphere(center, r, v0);
                col = Color::RED;
                projected = true;
            }
            break;
        case LINE_SPACE:
            if (!inCell(v0, cellOrigin, size)) {
                Vector3f v1;
                solveQEF(center, v1);
                if (inCell(v1, cellOrigin, size)) {
                    v0 = v1;
                } else if (intersect(cellOrigin, size, v0, v1)) {
                    col = Color::BLUE;
                } else {
                    opt = v0;
                    projectToSphere(center, r, v0);
                    col = Color::RED;
                    projected = true;
                }
            }
            break;
        case PLANE_SPACE:
            if (!inCell(v0, cellOrigin, size)) {
                Vector3f v1;
                solveQEF(center, v1);
                if (inCell(v1, cellOrigin, size)) {
                    v0 = v1;
                } else {
                    Vector3f v2;
                    solveQEF(cellOrigin, v2);
                    if (inCell(v2, cellOrigin, size)) {
                        v0 = v2;
                    } else if (intersect(cellOrigin, size, v0, v1, v2)) {
                        col = Color::YELLOW;
                    } else {
                        opt = v0;
                        projectToSphere(center, r, v0);
                        col = Color::RED;
                        projected = true;
                    }
                }
            }
            break;
        case UNDEFINED:
            return;
    }

    this->v = new Vertex(v0, col);
    if (projected)
        this->v_opt = new Vertex(opt, col);
    else
        this->v_opt = this->v;

}

double Cell::quadricError() const {
    if (qef)
        return qef->evaluate(v->position);
    return 0.0;
}

double Cell::optQuadricError() const {
    if (qef)
        return qef->evaluate(v_opt->position);
    return 0.0;
}

Vector3f Cell::backCut(const Edge& e, const uint i, const float size) const {
    float d = e.back_d*size; // distance from grid point 0..size
    Vector3f p = size*edge_origin[i].cast<float>();
    p[e.orientation] += d;
    return p;
}

Vector3f Cell::frontCut(const Edge& e, const uint i, const float size) const {
    float d = e.front_d*size; // distance from grid point 0..size
    Vector3f p = size*edge_origin[i].cast<float>();
    p[e.orientation] += d;
    return p;
}

void Cell::free() {
    delete qef;
    qef = nullptr;
    delete v;
    v = nullptr;
    delete v_opt;
    v_opt = nullptr;
}

Cell::~Cell() {
    free();
}

//==================================
//CellLeaf
//==================================

CellLeaf::CellLeaf(const int level) : Cell(level) {
}

bool CellLeaf::apply(const Vector3i& gridIndices, array<Edge,12>& edges) {
    // get global data
    dc::VertexGenerator& g = dc::VertexGenerator::getInstance();

    // get signs at cube corners
    for (int i = 0; i < 8; ++i) {
        p[i] = g.points[dc::id(gridIndices+point_origin[i])];
    }
    if (isInside())
        return false;


    // get edge data
    for (int i = 0; i < 12; ++i) {
        Vector3i index = gridIndices+edge_origin[i];
        // gather informations
        dc::edge(index, edge_orientation[i], edges[i]);
        edges[i].p[0] = p[edge_corners[i][0]];
        edges[i].p[1] = p[edge_corners[i][1]];
    }
    return true;
}

bool CellLeaf::calcQEF(const array<Edge,12>& edges, const Vector3f& origin, const float size) {
    vector<Vector3f*> normals;
    vector<Vector3f> p; // edge intersection points relative to cell origin

    for (int i = 0; i < 12; ++i) {
        const Edge& e = edges[i];
        // in   bf out
        // *-----|--O
        if (e.p[0] && !e.p[1]) {
            assert(e.hasBackCut);
            normals.push_back(e.back_normal);
            p.push_back(backCut(e, i, size));
        }
        //out ff    in
        // O--|-----*
        if (!e.p[0] && e.p[1]) {
            assert(e.hasFrontCut);
            normals.push_back(e.front_normal);
            p.push_back(frontCut(e, i, size));
        }
        //out ff bf out
        // O--|--|--O
        if (!e.p[0] && !e.p[1] && e.hasFrontCut && e.hasBackCut) {
            normals.push_back(e.front_normal);
            normals.push_back(e.back_normal);
            p.push_back(frontCut(e, i, size));
            p.push_back(backCut(e, i, size));
        }
    }

    if (p.size() > 0) {
        qef = new QEF();
        for (uint i = 0; i < p.size(); ++i) {
            qef->m += p[i];
        }
        qef->m /= p.size();
        qef->m += origin; //relative to grid origin
        for (uint i = 0; i < p.size(); ++i) {
            qef->add(*normals[i], p[i]+origin); // relative to grid origin
        }
        return true;
    }
    return false;
}

CellInfo CellLeaf::handle(const Vector3i& gridIndices, const float size) {
    array<Edge, 12> edges;
    Vector3f origin = gridIndices.cast<float>()*size;
    if(apply(gridIndices, edges) && calcQEF(edges, origin, size)) { // heterogeneous cell
        generateVertex(origin, size);
        if (optQuadricError() <= t)
             return HETEROGENEOUS;

        return FINAL_HETEROGENEOUS;
    }
    return HOMOGENEOUS;
}

bool CellLeaf::hasChildren() const {
    return false;
}

Cell* CellLeaf::getChild(const int i) const {
    return nullptr;
}

//==================================
//CellNode
//==================================

void CellNode::removeChildren() {
    if (hasChildren()) {
        for (int i = 0; i < 8; ++i) {
            delete children[i];
            children[i] = nullptr;
        }
    }
}

double CellNode::maxChildError() const {
    double E = 0.0;
    for (int i = 0; i < 8; ++i) {
        E = max(E, children[i]->quadricError());
    }
    return E;
}

CellInfo CellNode::unify(const bool homogeneous, const Vector3i& gridIndices, const float size) {
    dc::VertexGenerator& g = dc::VertexGenerator::getInstance();
    if (homogeneous) {
        bool sign = children[0]->p[0]; // all same sign
        for (int i = 0; i < 8; ++i) {
            p[i] = sign;
        }
        removeChildren();
        return HOMOGENEOUS;
    } else {

        // simplification
        qef = new QEF();

        // average masspoint from children QEFs with highest dimension
        int mCount = 0;
        for (int i = 0; i < 8; ++i) {
            if (children[i]->qef) {
                qef->add(*(children[i]->qef));
                if (qef->dimension == children[i]->qef->dimension) {
                    qef->m += children[i]->qef->m;
                    mCount++;
                } else if (qef->dimension < children[i]->qef->dimension) {
                    qef->m = children[i]->qef->m;
                    qef->dimension = children[i]->qef->dimension;
                    mCount = 1;
                }
            }
        }
        qef->m /= (float) mCount;
        Vector3f origin = gridIndices.cast<float>()*g.leafSize;
        generateVertex(origin, size);
        double E_v = quadricError();
        if (E_v <= t) {
            for (int i = 0; i < 8; ++i) {
                p[parent_child_corner[i]] = children[i]->p[parent_child_corner[i]];
            }
            removeChildren();
            return HETEROGENEOUS;
        }
        if (optQuadricError() <= t) {
            for (int i = 0; i < 8; ++i) {
                p[parent_child_corner[i]] = children[i]->p[parent_child_corner[i]];
            }
            // may combine children
            return HETEROGENEOUS;
        }

        free();
        return FINAL_HETEROGENEOUS;
    }
}

CellNode::CellNode(const int level) : Cell(level) {
    for (int i = 0; i < 8; ++i) {
        children[i] = nullptr;
    }
}

CellInfo CellNode::handle(const Vector3i& gridIndices, const float size) {

    if (level < leaf_level-1) {
        for (uint i = 0; i < 8; ++i) {
            children[i] = new CellNode(level+1);
        }       
    } else if (level == leaf_level-1) {
        for (uint i = 0; i < 8; ++i) {
            children[i] = new CellLeaf(level+1);
        }
    }
    int childGridSize = (res-1)/std::pow(2,level+1); // grid size of children cells


    bool homogeneous = true;
    bool merge = true;
    for (int i = 0; i < 8; ++i) {

        if (level == 1)
            std::cout << i << "..." << std::flush;
        if (level == 0)
            std::cout << i << ": " << std::flush;

        Vector3i childIndices = gridIndices + cell_origin[i]*childGridSize;
        switch (children[i]->handle(childIndices, size*0.5f)) {
            case HETEROGENEOUS:
                homogeneous = false;
                break;
            case HOMOGENEOUS:
                break;
            case FINAL_HETEROGENEOUS:
                homogeneous = false;
                merge = false;
                break;
        }
    }

    if (level <= 1)
        std::cout << std::endl;

    if (merge) {
        return unify(homogeneous, gridIndices, size);
    }

    return FINAL_HETEROGENEOUS;

}

bool CellNode::hasChildren() const {
    return children[0]; // either all or no children
}

Cell* CellNode::getChild(int i) const {
    return children[i];
}

CellNode::~CellNode() {
    removeChildren();
}

//==================================
//Dual Conturing
//==================================


namespace dc {

    uint id(const Vector3i& indices) {
        return (indices[2]*res+indices[1])*res+indices[0];
    }

    Vector3f* extract(const GLuint i, float& d) {
        d = 0;
        vector<byte> bytes;
        bytes.resize(4);
        uint x = i;
        for (int j = 3; j >= 0; --j) {
            if (x == 0)
                break;
            bytes[j] = (byte)(x & 255);
            x = x >> 8;
        }
        if (bytes[0] > 0) {
            //std::cout << (uint)bytes[0] << ", " << (uint)bytes[1] << ", " << (uint)bytes[2] << ", " << (uint)bytes[3] << std::endl;
            d = (float)(bytes[0]-1)/254.0; // distance from grid point 0..1
            return new Vector3f(bytes[1]*2/255.0 - 1, bytes[2]*2/255.0 - 1, bytes[3]*2/255.0 - 1);
        }

        return nullptr;
    }

    void edge(const Vector3i& index, const uint orientation, Edge& e) {
        // get global data
        VertexGenerator& g = VertexGenerator::getInstance();
        GLuint frontEdgeInfo = g.frontEdgeInfo(index, orientation);
        GLuint backEdgeInfo = g.backEdgeInfo(index, orientation);
        e.orientation = orientation;
        e.hasFrontCut = (e.front_normal = extract(frontEdgeInfo, e.front_d));
        e.hasBackCut = (e.back_normal = extract(backEdgeInfo, e.back_d));
    }

    //==================================
    //Vertex Generator
    //==================================


    VertexGenerator::VertexGenerator() : frontEdges(nullptr), backEdges(nullptr) {
    }

    VertexGenerator& VertexGenerator::getInstance() {
        static VertexGenerator instance;
        return instance;
    }

    void VertexGenerator::fill() {
        vector<bool> visited;
        visited.resize(res*res*res);
        queue<Vector3i> nextIndices;
        // push origin
        Vector3i origin = Vector3i(0,0,0);
        visited[id(origin)] = true;
        nextIndices.push(origin);

        while (!nextIndices.empty()) {
            // handle current
            Vector3i current = nextIndices.front();
            nextIndices.pop();
            points[id(current)] = false;
            // push next
            for (int i = 0; i < 6; ++i) {
                Vector3i next = current+neighbor_points[i];
                Vector3i edge = current+neighbor_edges[i];
                uint orientation = neighbor_edge_orientation[i];
                if (isEdge(edge, orientation) && !visited[id(next)] &&
                        frontEdgeInfo(edge, orientation) == 0 && backEdgeInfo(edge, orientation) == 0) {
                    nextIndices.push(next);
                    visited[id(next)] = true;
                }
            }
        }

    }

    CellNode* VertexGenerator::generate(const vector<vector<GLuint>>& frontEdges,
                       const std::vector<std::vector<GLuint>> &backEdges, const Vector3f& o, const float size) {
        CellNode* root = new CellNode(0);

        this->frontEdges = &frontEdges;
        this->backEdges = &backEdges;
        this->gridOrigin = o;
        this->rootSize = size;
        this->leafSize = size/(res-1);


        points.resize(res*res*res);
        for (uint i = 0; i < points.size(); ++i) {
            points[i] = true;
        }

        std::cout << "flood fill" << std::endl;
        fill();

        std::cout << "create octree and generate vertices for the final dual conturing mesh" << std::endl;
        root->handle(ORIGIN_GRID_INDICES, size);

        return root;
    }

    bool VertexGenerator::isEdge(const Vector3i& index, int orientation) const {
        int x = index[0];
        int y = index[1];
        int z = index[2];
        return x >= 0 && y >= 0 && z >= 0 && x < res && y < res && z < res && index[orientation] < res-1;
    }

    GLuint VertexGenerator::backEdgeInfo(const Vector3i& index, uint orientation) const {
        int x = index[0];
        int y = index[1];
        int z = index[2];

        switch (orientation) {
            case 0: // view in +x direction
                return (*backEdges)[orientation][(x*res+y)*res+z];
            case 1: // view in +y direction
                return (*backEdges)[orientation][(y*res+z)*res+x];
            case 2: // view in +z direction (local x = global -x)
                return (*backEdges)[orientation][(z*res+y)*res+(res-1-x)];
        }
        return 0;
    }

    GLuint VertexGenerator::frontEdgeInfo(const Vector3i& index, uint orientation) const {
        int x = index[0];
        int y = index[1];
        int z = index[2];

        switch (orientation) {
            case 0: // view in +x direction
                return (*frontEdges)[orientation][(x*res+y)*res+z];
            case 1: // view in +y direction
                return (*frontEdges)[orientation][(y*res+z)*res+x];
            case 2: // view in +z direction (local x = global -x)
                return (*frontEdges)[orientation][(z*res+y)*res+(res-1-x)];
        }
        return 0;
    }

    //==================================
    //Collector
    //==================================


    void Collector::collect(Cell* const cell, const Vector3i& gridIndices, const float size) {
        if (cell->hasChildren()) {
            int childGridSize = (res-1)/std::pow(2,cell->level+1); // grid size of children cells

            for (int i = 0; i < 8; ++i) {
               Vector3i childIndices = gridIndices + cell_origin[i]*childGridSize;
               collect(cell->getChild(i), childIndices, size*0.5f);
            }
        } else {
            dc::VertexGenerator& g = dc::VertexGenerator::getInstance();
            // collect generated vertex of cell
            if (cell->v) {
                cell->vertIndex = vertices.size();
                cell->v->position += g.gridOrigin; // relative to coord origin
                vertices.push_back(*(cell->v));
            }
            /*
            // collect edge cuts of cell
            for (int i = 0; i < 12; ++i) {
                shared_ptr<Edge> e = cell->edges[i];
                if (e->hasFrontCut)
                    edgeCuts.push_back(Vertex(cell->frontCut(i), *(e->front_normal)));
                if (e->hasBackCut)
                    edgeCuts.push_back(Vertex(cell->backCut(i), *(e->back_normal)));
            }
            */

            Vector3f origin = gridIndices.cast<float>()*g.leafSize;

            // calculate cell corners
            array<Vector3f, 8> corners;
            for (int i = 0; i < 8; ++i) {
                corners[i] = g.gridOrigin + origin + size*point_origin[i].cast<float>();
            }
            // collect cell contur
            for (int i = 0; i < 12; ++i) {
                cells.push_back(Vertex(corners[edge_corners[i][0]]));
                cells.push_back(Vertex(corners[edge_corners[i][1]]));
            }
            // collect cell corner signs
            for (int i = 0; i < 8; ++i) {
                if (cell->p[i])
                    grid.push_back(Vertex(corners[i], Color::GREEN));
            }

        }
    }

    //==================================
    //Index Generator
    //==================================

    bool IndexGenerator::checkNormal(const Cell* const q[4], initializer_list<int> normal_order, const Vector3f& dir) {
        const int* p = normal_order.begin();
        Vector3f n1 = (q[*(p++)]->v->position-q[*(p++)]->v->position)
               .cross(q[*(p++)]->v->position-q[*(p++)]->v->position);
        Vector3f n2 = (q[*(p++)]->v->position-q[*(p++)]->v->position)
               .cross(q[*(p++)]->v->position-q[*(p++)]->v->position);
        return n1.transpose() * dir >= 0 && n2.transpose() * dir >= 0;
    }

    void IndexGenerator::triangle(const Cell* const q[4], initializer_list<int> index_order, initializer_list<int> normal_order) {
        const int* p = index_order.begin();
        indices.push_back(q[*(p++)]->vertIndex);
        indices.push_back(q[*(p++)]->vertIndex);
        indices.push_back(q[*(p++)]->vertIndex);
        p = normal_order.begin();
        Vector3f n = (q[*(p++)]->v->position-q[*(p++)]->v->position)
                .cross(q[*(p++)]->v->position-q[*(p++)]->v->position);
        n.normalize();
        normals.push_back(n);
    }

    void IndexGenerator::triangulate(const Cell* const q[4], bool front_face, const uint orientation) {
        Vector3f dir(0,0,0);
        dir[orientation] = front_face? 1 : -1;
        float d0_2 = (q[0]->v->position - q[2]->v->position).squaredNorm();
        float d1_3 = (q[1]->v->position - q[3]->v->position).squaredNorm();
        if (front_face) {
            if ((d0_2 < d1_3 && checkNormal(q, {1,0,2,0, 2,0,3,0}, dir)) ||
                (d0_2 >= d1_3 && !checkNormal(q, {1,0,3,0, 2,1,3,1}, dir))) {
                triangle(q, {0,1,2}, {1,0,2,0});
                triangle(q, {0,2,3}, {2,0,3,0});
            } else {
                triangle(q, {0,1,3}, {1,0,3,0});
                triangle(q, {1,2,3}, {2,1,3,1});
            }
        } else {
            if ((d0_2 < d1_3 && checkNormal(q, {2,0,1,0, 3,0,2,0}, dir)) ||
                (d0_2 >= d1_3 && !checkNormal(q, {3,0,1,0, 3,1,2,1}, dir))) {
                triangle(q, {0,2,1}, {2,0,1,0});
                triangle(q, {0,3,2}, {3,0,2,0});
            } else {
                triangle(q, {0,3,1}, {3,0,1,0});
                triangle(q, {1,3,2}, {3,1,2,1});
            }
        }
    }

    void IndexGenerator::edgeProc(const Cell* const q[4], const uint orientation) {
        if (q[0]->hasChildren() || ((q[1]) && q[1]->hasChildren()) || ((q[2]) && q[2]->hasChildren()) || ((q[3]) && q[3]->hasChildren())) {
            for (int i = 0; i < 2; ++i) {
                const Cell* childs[4];
                for (int j = 0; j < 4; ++j) {
                    if (q[j] && q[j]->hasChildren()) {
                        childs[j] = q[j]->getChild(edge_childs_of_edge[orientation][i][j]);
                    } else {
                        childs[j] = q[j];
                    }
                }
                edgeProc(childs, orientation);
            }
        } else {
            // identify edge

            int maxdepth = -1;
            int maxDepthCellIndex = -1;
            for (int i = 0; i < 4; ++i) {
                if (q[i] && (q[i]->level > maxdepth)) {
                    maxDepthCellIndex = i;
                    maxdepth = q[i]->level;
                }
            }
            int edgeIndex = edge_of_four_cells[orientation][maxDepthCellIndex];
            bool p_0 = q[maxDepthCellIndex]->p[edge_corners[edgeIndex][0]];
            bool p_1 = q[maxDepthCellIndex]->p[edge_corners[edgeIndex][1]];


            // generate quad or triangle?
            bool quad = q[0] && q[1] && q[2] && q[3];

            if (!p_0 && p_1) { // frontface

                if (quad) {
                    if (!q[0]->v || !q[1]->v || !q[2]->v || !q[3]->v) {
                        std::cout << "null_quad" << std::endl;
                        return;
                    }
                    triangulate(q, true, orientation);

                } else { // triangle only
                    for (int i = 0; i < 4; ++i) {
                        if (q[i] && !q[i]->v) {
                            std::cout << "null_triangle" << std::endl;
                            return;
                        }
                    }
                    for (int i = 0; i < 4; ++i) {
                        if (q[i])
                            indices.push_back(q[i]->vertIndex);
                    }
                    // calculate normal
                    Vector3f v[3];
                    int count = 0;
                    for (int i = 0; i < 4; ++i) {
                        if (q[i]) {
                            v[count++] = (q[i]->v->position);
                        }
                    }
                    Vector3f n = (v[0]-v[2]).cross(v[1]-v[2]);
                    n.normalize();
                    normals.push_back(n);
                }
            }
            if (p_0 && !p_1) { // backface
                if (quad) {
                    if (!q[0]->v || !q[1]->v || !q[2]->v || !q[3]->v) {
                        std::cout << "null_quad" << std::endl;
                        return;
                    }
                    triangulate(q, false, orientation);
                } else { // triangle only
                    for (int i = 0; i < 4; ++i) {
                        if (q[i] && !q[i]->v) {
                            std::cout << "null_triangle" << std::endl;
                            return;
                        }
                    }
                    for (int i = 3; i >= 0; --i) {
                        if (q[i])
                            indices.push_back(q[i]->vertIndex);
                    }
                    Vector3f v[3];
                    int count = 0;
                    for (int i = 0; i < 4; ++i) {
                        if (q[i]) {
                            v[count++] = (q[i]->v->position);
                        }
                    }
                    Vector3f n = (v[2]-v[0]).cross(v[1]-v[2]);
                    n.normalize();
                    normals.push_back(n);
                }
            }
        }
    }

    /**
     * @brief faceProc
     * @param q1
     * @param q2
     * @param orientation 0=yz plane, 1=xz plane 2=xy plane
     */
    void IndexGenerator::faceProc(const Cell* const q[2], const uint face_orientation) {
        if (q[0]->hasChildren() || q[1]->hasChildren()) {
            for (int i = 0; i < 4; ++i) { // 4 faces tiling coarse face
                const Cell* childs[2];
                for (int j = 0; j < 2; ++j) { // 2 child cells define 1 face
                    if (q[j]->hasChildren())
                        childs[j] = q[j]->getChild(face_childs_of_face[face_orientation][i][j]);
                    else
                        childs[j] = q[j]; // use coarse cell
                }
                faceProc(childs, face_orientation);
            }
            for (int i = 0; i < 4; ++i) { // 4 edges in coarse face
                const Cell* childs[4];
                // specifies whether the coarse cell was already added to childs[] (only in case q[0] or q[1] has no children)
                bool coarseCellAdded = false;

                for (int j = 0; j < 4; ++j) { // 4 child cells define 1 edge
                    if (q[edge_childs_of_face[face_orientation][i][j][0]]->hasChildren()) {
                        childs[j] = q[edge_childs_of_face[face_orientation][i][j][0]]
                                    ->getChild(edge_childs_of_face[face_orientation][i][j][1]);
                    } else if (coarseCellAdded)
                        childs[j] = nullptr; // do not add coarse cell again
                    else {
                        childs[j] = q[edge_childs_of_face[face_orientation][i][j][0]]; // use coarse cell (first time)
                        coarseCellAdded = true;
                    }
                }
                edgeProc(childs, face_edge_orientation[face_orientation][i]);
            }
        }
    }

    void IndexGenerator::cellProc(const Cell* const q) {
        if (q->hasChildren()) {
            // inner cells
            for (int i = 0; i < 8; ++i) {
                cellProc(q->getChild(i));
            }
            // inner faces
            for (int i = 0; i < 3; ++i) { // 3 main inner planes (yz, xz, xy)
                for (int j = 0; j < 4; ++j) { // each tiled to 4 faces
                    const Cell* childs[2]
                        {q->getChild(face_childs_of_cell[i][j][0]), q->getChild(face_childs_of_cell[i][j][1])};
                    faceProc(childs, i);
                }
            }
            // inner edges
            for (int i = 0; i < 3; ++i) { // 3 edge orientations (x, y, z)
                for (int j = 0; j < 2; ++j) { // each divided by 2
                    const Cell* childs[4]
                        {q->getChild(edge_childs_of_cell[i][j][0]), q->getChild(edge_childs_of_cell[i][j][1]),
                         q->getChild(edge_childs_of_cell[i][j][2]), q->getChild(edge_childs_of_cell[i][j][3])};
                    edgeProc(childs, i);
                }
            }
        }

    }

    void IndexGenerator::generate(Cell* const root) {
        cellProc(root);
    }

    //==================================
    //Mesh Generator
    //==================================

    void MeshGenerator::generate(const vector<Vertex>& vertices, const vector<uint>& indices, const vector<Vector3f>& normals) {
        this->vertices.reserve(indices.size());
        uint i = 0;
        foreach (const Vector3f n, normals) {
            Vertex v = vertices[indices[i++]];
            this->vertices.push_back(Vertex(v.position, n, v.color));
            v = vertices[indices[i++]];
            this->vertices.push_back(Vertex(v.position, n, v.color));
            v = vertices[indices[i++]];
            this->vertices.push_back(Vertex(v.position, n, v.color));
        }
    }

    void MeshGenerator::generate(const vector<vector<GLuint>>& frontEdges,
                                 const std::vector<std::vector<GLuint>> &backEdges, const Vector3f& o, const float size) {
        dc::VertexGenerator& g = dc::VertexGenerator::getInstance();
        CellNode* root = g.generate(frontEdges, backEdges, o, size);
        std::cout << "collect generated data from octree" << std::endl;
        collector.collect(root, ORIGIN_GRID_INDICES, size);
        std::cout << "generate indices for the dual conturing mesh" << std::endl;
        iGenerator.generate(root);
        generate(collector.vertices, iGenerator.indices, iGenerator.normals);
    }

    const vector<Vertex>& MeshGenerator::getVertices() const {
        return this->vertices;
    }
}

