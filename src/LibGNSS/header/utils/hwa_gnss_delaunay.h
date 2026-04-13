#ifndef hwa_gnss_base_celaunay_H
#define hwa_gnss_base_celaunay_H

#include "hwa_base_eigendef.h"

using namespace hwa_base;

namespace hwa_gnss
{
    /** @brief struct for float 2D-point. */
    struct point_float
    {
        double x;
        double y;
        //char station[4];
        int indx;
    };

    /** @brief struct for float 2D-points. */
    struct points3_ABC
    {
        point_float A;
        point_float B;
        point_float C;
    };

    /** @brief vertex2d type. */
    typedef struct VERTEX2D_TYP
    {
        double x;
        double y;
        //string station;
        char station[4];
        int indx;

    } VERTEX2D, *VERTEX2D_PTR;
    
    /** @brief edge type. */
    typedef struct EDGE_TYP
    {
        VERTEX2D v1;
        VERTEX2D v2;

    } EDGE, *EDGE_PTR;
    
    /** @brief triangle type. */
    typedef struct TRIANGLE_TYP
    {
        int i1; // vertex index
        int i2;
        int i3;

        TRIANGLE_TYP *pNext;
        TRIANGLE_TYP *pPrev;

    } TRIANGLE, *TRIANGLE_PTR;
    
    /** @brief mesh type. */
    typedef struct MESH_TYP
    {
        int vertex_num;
        int triangle_num;

        VERTEX2D_PTR pVerArr;    ///< point to outer vertices arrary
        TRIANGLE_PTR pTriArr;    //< point to outer triangles arrary
    } MESH, *MESH_PTR;


    /**
    * @brief virtual class for delaunay method
    */
    class gnss_base_celaunay
    {
    public:
        /** @brief default constructor. */
        explicit gnss_base_celaunay();

        /** @brief default destructor. */
        virtual ~gnss_base_celaunay();

        /**
        * @brief get init site.
        *
        * @param[in]  x_r         todo
        * @param[in]  posList      position list
        * @param[in]  outIndx      out index
        * @return      int          get mode
        *
        */
        int getIntSite(Triple &x_r, std::vector<Triple> &posList, int outIndx[3]);

    private:

        /**
        * @brief init mesh.
        *
        * @param[in]  x_r         todo
        * @param[in]  pMesh       mesh pointer
        * @param[in]  ver_num      ver number
        * @return      void
        *
        */
        void InitMesh(MESH_PTR pMesh, int ver_num);

        /**
        * @brief Uninit mesh.
        *
        * @param[in]  pMesh       mesh pointer
        * @return      void
        *
        */
        void UnInitMesh(MESH_PTR pMesh);

        /**
        * @brief add bounding box.
        *
        * @param[in]  pMesh       mesh pointer
        * @return      void
        *
        */
        void AddBoundingBox(MESH_PTR pMesh);

        /**
        * @brief remove bounding box.
        *
        * @param[in]  pMesh       mesh pointer
        * @return      void
        *
        */
        void RemoveBoundingBox(MESH_PTR pMesh);

        /**
        * @brief counter clock wise.
        *
        * @param[in]  pa           VERTEX2D pointer
        * @param[in]  pb           VERTEX2D pointer
        * @param[in]  pc           VERTEX2D pointer
        * @return      double      the result
        *
        */
        double CounterClockWise(VERTEX2D_PTR pa, VERTEX2D_PTR pb, VERTEX2D_PTR pc);

        /**
        * @brief incremental Delaunay.
        *
        * @param[in]  pMesh       mesh pointer
        * @return      void
        *
        */
        void IncrementalDelaunay(MESH_PTR pMesh);

        /**
        * @brief insert.
        *
        * @param[in]  pMesh       mesh pointer
        * @param[in]  ver_index   ver index
        * @return      void
        *
        */
        void Insert(MESH_PTR pMesh, int ver_index);

        /**
        * @brief flip test.
        *
        * @param[in]  pMesh       mesh pointer
        * @param[in]  pTestTri      test Triangle pointer
        * @return      bool          test result
        *
        */
        bool FlipTest(MESH_PTR pMesh, TRIANGLE_PTR pTestTri);

        /**
        * @brief judege in circle.
        *
        * @param[in]  pMesh       mesh pointer
        * @param[in]  pa           VERTEX2D pointer
        * @param[in]  pb           VERTEX2D pointer
        * @param[in]  pp           VERTEX2D pointer
        * @param[in]  pd           VERTEX2D pointer
        * @return      double      the result
        *
        */
        double InCircle(VERTEX2D_PTR pa, VERTEX2D_PTR pb, VERTEX2D_PTR pp, VERTEX2D_PTR pd);

        /**
        * @brief judege in Triangle.
        *
        * @param[in]  pMesh       mesh pointer
        * @param[in]  pVer           VERTEX2D pointer
        * @param[in]  pTri           Triangle pointer
        * @return      double      the result
        *
        */
        double InTriangle(MESH_PTR pMesh, VERTEX2D_PTR pVer, TRIANGLE_PTR pTri);

        /**
        * @brief insert in Triangle.
        *
        * @param[in]  pMesh       mesh pointer
        * @param[in]  pTargetTri  target Triangle pointer
        * @param[in]  ver_index      ver index
        * @return      void
        *
        */
        void InsertInTriangle(MESH_PTR pMesh, TRIANGLE_PTR pTargetTri, int ver_index);

        /**
        * @brief insert on edge.
        *
        * @param[in]  pMesh       mesh pointer
        * @param[in]  pTargetTri  target Triangle pointer
        * @param[in]  ver_index      ver index
        * @return      void
        *
        */
        void InsertOnEdge(MESH_PTR pMesh, TRIANGLE_PTR pTargetTri, int ver_index);

        /**
        * @brief remove Triangle mode.
        *
        * @param[in]  pMesh       mesh pointer
        * @param[in]  pTargetTri  target Triangle pointer
        * @return      void
        *
        */
        void RemoveTriangleNode(MESH_PTR pMesh, TRIANGLE_PTR pTri);

        /**
        * @brief add Triangle mode.
        *
        * @param[in]  pMesh       mesh pointer
        * @param[in]  pTargetTri  target Triangle pointer
        * @param[in]  i1,i2,i3      int          
        * @return      Triangle      pointer
        *
        */
        TRIANGLE_PTR AddTriangleNode(MESH_PTR pMesh, TRIANGLE_PTR pPrevTri, int i1, int i2, int i3);

        /**
        * @brief input.
        *
        * @param[in]  pFile       file pointer
        * @param[in]  pMesh          mesh pointer
        * @return      void
        *
        */
        void Input(char *pFile, MESH_PTR pMesh);

        /**
        * @brief input position.
        *
        * @param[in]  posList       position list
        * @param[in]  pMesh          mesh pointer
        * @return      void
        *
        */
        void Input_pos(std::vector<Triple> &posList, MESH_PTR pMesh);

        /**
        * @brief output.
        *
        * @param[in]  pMesh          mesh pointer
        * @param[in]  point       float point
        * @return      points3_ABC
        *
        */
        points3_ABC Output(MESH_PTR pMesh, point_float Point);

        /**
        * @brief judege whether in Triangle.
        *
        * @param[in]  A, B, C, D    float points
        * @return      int            result
        *
        */
        int IsInTriangle(point_float A, point_float B, point_float C, point_float D);

        /**
        * @brief get triangle squar.
        *
        * @param[in]  pf0, pf1, pf2        float points
        * @return      double            result
        *
        */
        double GetTriangleSquar(point_float pf0, point_float pf1, point_float pf2);
    };

}
#endif