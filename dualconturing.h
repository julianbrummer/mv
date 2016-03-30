#ifndef DUALCONTURING
#define DUALCONTURING

#include <GL/gl.h>
#include <QVector3D>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <cmath>
#include <memory>
#include <map>
#include <deque>

#include <QGLShaderProgram>

typedef uint8_t byte;

using namespace Eigen;
using namespace std;

const int res = 129; // render resolution = grid resolution + 1
const int leaf_level = 7;
const float eps = 1E-5;
const float t = 0.001; // simplification threshold
const float truncation = 0.01; // singular value truncation

// shift of child origin from parent origin
const Vector3i cell_origin[8] = {
    Vector3i(0,0,0), Vector3i(1,0,0), Vector3i(1,1,0), Vector3i(0,1,0),
    Vector3i(0,0,1), Vector3i(1,0,1), Vector3i(1,1,1), Vector3i(0,1,1)
};

// shift of grid point origin from cell origin
const Vector3i point_origin[8] = {
    Vector3i(0,0,0), Vector3i(1,0,0), Vector3i(1,0,1), Vector3i(0,0,1),
    Vector3i(0,1,0), Vector3i(1,1,0), Vector3i(1,1,1), Vector3i(0,1,1)
};

// shift of edge origin from cell origin
const Vector3i edge_origin[12] = {
    Vector3i(0,0,0), Vector3i(1,0,0), Vector3i(0,0,1), Vector3i(0,0,0),
    Vector3i(0,1,0), Vector3i(1,1,0), Vector3i(0,1,1), Vector3i(0,1,0),
    Vector3i(0,0,0), Vector3i(1,0,0), Vector3i(1,0,1), Vector3i(0,0,1)
};

// plane orientations of a cube (0=x, 1=y, 2=z)
const uint plane_orientation[6] = {
    0,0, // (yz plane)
    1,1, // (xz plane)
    2,2  // (xy plane)
};

// shift of cell plane from cell origin
const uint plane_shift[6] = {
    0,1, // (yz plane)
    0,1, // (xz plane)
    0,1  // (xy plane)
};

// edge orientations of a cube (0=x, 1=y, 2=z)
const uint edge_orientation[12] = {
    0,2,0,2,0,2,0,2,1,1,1,1
};

// edge defined by 2 corners of the cube
const uint edge_corners[12][2] = {
    {0,1},{1,2},{3,2},{0,3},
    {4,5},{5,6},{7,6},{4,7},
    {0,4},{1,5},{2,6},{3,7}
};

// for each parent edge {child cell (1st half edge), child cell (2nd half edge)}
const uint edge_split[12][2] = {
    {0,1}, {1,5}, {4,5}, {0,4},
    {3,2}, {2,6}, {7,6}, {3,7},
    {0,3}, {1,2}, {5,6}, {4,7}
};

// corner index in child i of parent corner
const uint parent_child_corner[8] = {
    0,1,5,4,3,2,6,7
};

// shift for each virtual inner edge of parent cell (in child cell size units) (0..2 orientation, 0..4 inner edges)
const Vector3i inner_edge_shift[3][5] = {
    {Vector3i(0,0,1), Vector3i(0,1,1), Vector3i(0,2,1), Vector3i(0,1,0), Vector3i(0,1,2)},
    {Vector3i(0,0,1), Vector3i(1,0,1), Vector3i(2,0,1), Vector3i(1,0,0), Vector3i(1,0,2)},
    {Vector3i(0,1,0), Vector3i(1,1,0), Vector3i(2,1,0), Vector3i(1,0,0), Vector3i(1,2,0)},
};
// for each orientation 0..2, for each virtual inner edge (1st half) of parent cell 0..4 {{child cell, edge index}, ...}
const int fst_child_edges[3][5][4][2] = {
    {{{0,4}, {3,0}, {-1,-1}, {-1,-1}}, {{0,6}, {3,2}, {4,4}, {7,0}},
     {{4,6}, {7,2}, {-1,-1}, {-1,-1}}, {{0,2}, {4,0}, {-1,-1}, {-1,-1}}, {{3,6}, {7,4}, {-1,-1}, {-1,-1}}},

    {{{0,9}, {1,8}, {-1,-1}, {-1,-1}}, {{0,10}, {1,11}, {4,9}, {5,8}},
     {{4,10}, {5,11}, {-1,-1}, {-1,-1}}, {{0,11}, {4,8}, {-1,-1}, {-1,-1}}, {{1,10}, {5,9}, {-1,-1}, {-1,-1}}},

    {{{0,1}, {1,3}, {-1,-1}, {-1,-1}}, {{0,5}, {1,7}, {2,3}, {3,1}},
     {{2,7}, {3,5}, {-1,-1}, {-1,-1}}, {{0,7}, {3,3}, {-1,-1}, {-1,-1}}, {{1,5}, {2,1}, {-1,-1}, {-1,-1}}}
};

// for each orientation 0..2, for each virtual inner edge (2nd half) of parent cell 0..4 {{child cell, edge index}, ...}
const int snd_child_edges[3][5][4][2] = {
    {{{2,0}, {1,4}, {-1,-1}, {-1,-1}}, {{1,6}, {2,2}, {5,4}, {6,0}},
     {{5,6}, {6,2}, {-1,-1}, {-1,-1}}, {{1,2}, {5,0}, {-1,-1}, {-1,-1}}, {{2,6}, {6,4}, {-1,-1}, {-1,-1}}},

    {{{2,8}, {3,9}, {-1,-1}, {-1,-1}}, {{2,11}, {3,10}, {6,8}, {7,9}},
     {{6,11}, {7,10}, {-1,-1}, {-1,-1}}, {{3,11}, {7,8}, {-1,-1}, {-1,-1}}, {{2,10}, {6,9}, {-1,-1}, {-1,-1}}},

    {{{4,1}, {5,3}, {-1,-1}, {-1,-1}}, {{4,5}, {5,7}, {6,3}, {7,1}},
     {{6,7}, {7,5}, {-1,-1}, {-1,-1}}, {{4,7}, {7,3}, {-1,-1}, {-1,-1}}, {{5,5}, {6,1}, {-1,-1}, {-1,-1}}}
};

// adjacent child cells containing the same inner face of a cell
// [plane/orientation][inner_face][child_cells]
const uint face_childs_of_cell[3][4][2] {
    {{3,2}, {0,1}, {4,5}, {7,6}}, // x oriented child pairs (yz plane)
    {{4,7}, {0,3}, {1,2}, {5,6}}, // y oriented child pairs (xz plane)
    {{2,6}, {1,5}, {0,4}, {3,7}}  // z oriented child pairs (xy plane)
};

// adjacent child cells containing the same quater face of a face
// [plane/orientation][quater_face][child_cells(fst,snd)]
const uint face_childs_of_face[3][4][2] {
    {{2,3}, {1,0}, {5,4}, {6,7}}, // (yz plane)
    {{7,4}, {3,0}, {2,1}, {6,5}}, // (xz plane)
    {{6,2}, {5,1}, {4,0}, {7,3}}  // (xy plane)
};

// child cells containing the same inner edge of a cell
// [orientation][inner_edge][child_cells]
const uint edge_childs_of_cell[3][2][4] {
    {{3,0,4,7}, {2,1,5,6}}, // x oriented edges
    {{4,0,1,5}, {7,3,2,6}}, // y oriented edges
    {{2,1,0,3}, {6,5,4,7}}  // z oriented edges
};

// child cells containing the same edge of a tiled face
// [plane/orientation][edge][child_cells][coarse_cell(0,1),child_cell_index]
const uint edge_childs_of_face[3][4][4][2] {
    {{{0,5},{0,1},{1,0},{1,4}}, {{0,6},{0,2},{1,3},{1,7}}, {{1,3},{1,0},{0,1},{0,2}}, {{1,7},{1,4},{0,5},{0,6}}}, // (yz plane)
    {{{1,0},{0,3},{0,7},{1,4}}, {{1,1},{0,2},{0,6},{1,5}}, {{1,1},{0,2},{0,3},{1,0}}, {{1,5},{0,6},{0,7},{1,4}}}, // (xz plane)
    {{{0,7},{0,4},{1,0},{1,3}}, {{0,6},{0,5},{1,1},{1,2}}, {{1,0},{0,4},{0,5},{1,1}}, {{1,3},{0,7},{0,6},{1,2}}}  // (xy plane)
};

// orientation of inner edges of a face
// [face_orientation][edge_orientation(0..2)]
const uint face_edge_orientation[3][4] {
    {1,1,2,2}, // (yz plane)
    {0,0,2,2}, // (xz plane)
    {0,0,1,1}  // (xy plane)
};
// child cells containing the same edge of a divided edge
//[plane/orientation][edge(fst,snd)][adjacent_child_cells(l_up, l_down, r_down, r_up)]
const uint edge_childs_of_edge[3][2][4] {
    {{4,7,3,0},{5,6,2,1}}, // x oriented edges
    {{1,5,4,0},{2,6,7,3}}, // y oriented edges
    {{0,3,2,1},{4,7,6,5}}  // z oriented edges
};
// edge indices defining the same edge defined by 4 cells
// [orientation][cell] -> edge index
const uint edge_of_four_cells[3][4] {
    {2,6,4,0}, // x oriented edges
    {9,10,11,8}, // y oriented edges
    {3,7,5,1}  // z oriented edges
};

// neighboring grid points
const Vector3i neighbor_points[6] {
    Vector3i(1,0,0), Vector3i(-1,0,0),
    Vector3i(0,1,0), Vector3i(0,-1,0),
    Vector3i(0,0,1), Vector3i(0,0,-1)
};

// neighboring edges
const Vector3i neighbor_edges[6] {
    Vector3i(0,0,0), Vector3i(-1,0,0),
    Vector3i(0,0,0), Vector3i(0,-1,0),
    Vector3i(0,0,0), Vector3i(0,0,-1)
};

// orientation of neighboring edges
const uint neighbor_edge_orientation[6] {
    0, 0,
    1, 1,
    2, 2
};

namespace Color {

    const Vector3f WHITE = Vector3f(1,1,1);
    const Vector3f GRAY = Vector3f(0.5,0.5,0.5);
    const Vector3f BLUE = Vector3f(0,0,1);
    const Vector3f YELLOW = Vector3f(1,1,0);
    const Vector3f GREEN = Vector3f(0,1,0);
    const Vector3f RED = Vector3f(1,0,0);
}

enum CellInfo {
    HOMOGENEOUS, // the entire cell is "in" or "out" and has no children
    HETEROGENEOUS, // the cell is neither "in" nor "out", may have children and could be merged
    FINAL_HETEROGENEOUS // the cell is heterogeneous, may have children and can not be merged
};

enum SolutionSpace {
    POINT_SPACE = 3,
    LINE_SPACE = 2,
    PLANE_SPACE = 1,
    UNDEFINED = 0 // not yet calculated
};

struct Vertex {
    Vector3f position;
    Vector3f normal;
    Vector3f color;

    Vertex(const Vector3f& position, const Vector3f& color = Color::WHITE);
    Vertex(const Vector3f& position, const Vector3f& normal, const Vector3f& color);
    Vertex() = default;
};
struct Edge {
    bool p[2];
    Vector3f* front_normal;
    Vector3f* back_normal;
    float front_d, back_d; // cut distance from edge origin 0..1
    int orientation; // x = 0, y = 1, z = 2
    bool hasFrontCut, hasBackCut;

    Edge();
    Edge(int orientation);

    bool isBoundary(uint size);
    void split(Edge& fst, Edge& snd);

    ~Edge();
};


class QEF {
private:
    array<double,6> a;
    Vector3d b;
    double c;
public:
    SolutionSpace dimension;
    Vector3f m; // the mass center of edge intersections relative to grid origin

    QEF();

    void add(const Vector3f& normal, const Vector3f& point);
    void add(const QEF& qef);
    SolutionSpace solve(const Vector3f &m, Vector3f& c);
    SolutionSpace solve(Vector3f& c);
    double evaluate(const Vector3f& v) const;

};

class Cell {
private:
    bool inCell(Vector3f& pos, const Vector3f& cellOrigin, const float size) const;
    SolutionSpace solveQEF(const Vector3f& m, Vector3f& v) const;
    SolutionSpace solveQEF(Vector3f& v) const;
    void projectToSphere(const Vector3f& cellOrigin, const float size, Vector3f& v) const;
    float intersect(const Vector3f& n, const Vector3f& p, const Vector3f& c, const uint orientation) const;
protected:

    void generateVertex(const Vector3f& cellOrigin, const float size);
    Vector3f backCut(const Edge& e, const uint i, const float size) const;
    Vector3f frontCut(const Edge& e, const uint i, const float size) const;
    void free();

public:
    double quadricError() const;
    double optQuadricError() const;
    bool intersect(const Vector3f& cellOrigin, const float size, Vector3f& v0, const Vector3f& v1) const;
    bool intersect(const Vector3f& cellOrigin, const float size, Vector3f& v0, const Vector3f& v1, const Vector3f& v2) const;
    bool p[8];
    Vertex* v; // the created vertex (in cell)
    Vertex* v_opt; // the optimal vertex (may lay outside from cell)
    int vertIndex;
    int level;
    QEF* qef;

    Cell(const int level);

    bool isInside() const;

    virtual bool hasChildren() const = 0;
    virtual Cell* getChild(int i) const = 0;
    virtual ~Cell();

    virtual CellInfo handle(const Vector3i& gridIndices, const float size) = 0;


};

class CellNode : public Cell {

private:
    Cell* children[8];

    void removeChildren();
    CellInfo unify(const bool homogeneous, const Vector3i& gridIndices, const float size);
    double maxChildError() const;

public:

    CellNode(const int level);
    CellInfo handle(const Vector3i& gridIndices, const float size);
    bool hasChildren() const;
    Cell* getChild(int i) const;

    ~CellNode();
};

class CellLeaf : public Cell {

private:
    bool apply(const Vector3i& gridIndices, array<Edge,12>& edges);
    bool calcQEF(const array<Edge,12>& edges, const Vector3f& origin, const float size);

public:

    CellLeaf(const int level);
    CellInfo handle(const Vector3i& gridIndices, const float size);
    bool hasChildren() const;
    Cell* getChild(int i) const;

};


/*

               *--------**--------*
              /|       /|        /|
             / |      / |       / |
            /  |     /  |      /  |
           *--------**--------*   |
          /|   |  3/|   |   2/|   |
         / |   *--/-|---**--/-|---*
        /  |  /| /  |  /|  /  |  /|
       *--------**--------*   | / |
       |   |/  ||   |/  | |   |/  |
       |   *----|---**----|---*   |
       |  /| 7 ||  /|  6| |  /|   |
       | / |   *|-/-|---**|-/-|---*
       |/  |  / |/0 |  /  |/1 |  /
       *--------**--------*   | /
       |   |/   |   |/    |   |/
       |   *----|---**----|---*
       |  /  4  |  /   5  |  /
       | /      | /       | /
       |/       |/        |/
       *--------**--------*

       *---4----*
      /|       /|
     7 |      5 |
    /  8     /  9
   *----6---*   |
   |   |    |   |
   |   O---0|---*
   11 /     10 /
   | 3      | 1
   |/       |/
   *---2----*

       4--------5
      /|       /|
     / |      / |
    /  |     /  |
   7--------6   |
   |   |    |   |
   |   0----|---1
   |  /     |  /
   | /      | /
   |/       |/
   3--------2

*/
namespace dc {
    static Vector3i ORIGIN_GRID_INDICES(0,0,0);

    uint id(const Vector3i &indices);
    Vector3f* extract(const GLuint i, float& d);
    void edge(const Vector3i& index, const uint orientation, Edge& e);

    class VertexGenerator {
    private:

        VertexGenerator();
    public:
        float leafSize, rootSize;
        Vector3f gridOrigin;
        vector<bool> points;
        const vector<vector<GLuint>>* frontEdges;
        const vector<vector<GLuint>>* backEdges;

        VertexGenerator(const VertexGenerator&) = delete;
        void operator =(const VertexGenerator&) = delete;

        static VertexGenerator& getInstance();

        void fill();
        CellNode* generate(const vector<vector<GLuint>>& frontEdges,
                           const vector<vector<GLuint>>& backEdges, const Vector3f& o, const float size);



        bool isEdge(const Vector3i& index, int orientation) const;
        GLuint frontEdgeInfo(const Vector3i& index, uint orientation) const;
        GLuint backEdgeInfo(const Vector3i& index, uint orientation) const;
    };

    class Collector {
    public:
        vector<Vertex> vertices;
        vector<Vertex> edgeCuts;
        vector<Vertex> cells;
        vector<Vertex> grid;

        void collect(Cell* const cell, const Vector3i& gridIndices, const float size);
    };

    class IndexGenerator {
    private:
        bool checkNormal(const Cell* const q[4], initializer_list<int> index_order, const Vector3f& dir);
        void triangle(const Cell* const q[4], initializer_list<int> index_order, initializer_list<int> normal_order);
        void triangulate(const Cell* const q[4], bool front_face, const uint orientation);
        void edgeProc(const Cell* const q[4], const uint orientation);
        void faceProc(const Cell* const q[2], const uint face_orientation);
        void cellProc(const Cell* const q);
    public:
        vector<uint> indices;
        vector<Vector3f> normals;
        void generate(Cell * const cell);
    };

    class MeshGenerator {
        vector<Vertex> vertices;

        void generate(const vector<Vertex>& vertices, const vector<uint>& indices, const vector<Vector3f>& normals);
    public:
        Collector collector;
        IndexGenerator iGenerator;

        void generate(const vector<vector<GLuint> > &frontEdges, const std::vector<std::vector<GLuint> > &backEdges, const Vector3f &o, const float size);
        const vector<Vertex>& getVertices() const;

    };
}

#endif // DUALCONTURING

