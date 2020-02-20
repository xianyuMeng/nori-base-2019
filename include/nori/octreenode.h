#pragma once

#include<nori/mesh.h>



NORI_NAMESPACE_BEGIN

class OctreeBaseNode
{
   public:
   OctreeBaseNode()
   {
       for(size_t i = 0; i < 8; ++i)
       {
           this->children[i] = nullptr;
       }
   } 
   virtual ~OctreeBaseNode(){};   
   OctreeBaseNode* children[8];
   OctreeBaseNode* parent = nullptr;
   BoundingBox3f m_bbox;
   std::vector<int> m_triangle_idx;
   int child_id = 0;
   int depth = 0;
   bool visited = false;
};

//OctreeBaseNode::~OctreeBaseNode()
//{
//    for(size_t i = 0; i < 8; ++i)
//    {
//        if(this->children[i])
//        {
//            delete this->children[i];
//        }
//    } 
//
//}

class OctreeNode : public OctreeBaseNode
{
    public:
    
    OctreeNode(const BoundingBox3f& bbox, std::vector<int>& triangle_idx)
    {
        m_bbox = bbox;
        m_triangle_idx = triangle_idx;
        this->init_child();
    }
    void init_child()
    {
        for(size_t i = 0; i < 8; ++i)
        {
            this->children[i] = nullptr;
        }
    }

    ~OctreeNode()
    {
        for(size_t i = 0; i < 8; ++i)
        {
            if(this->children[i])
            {
                delete this->children[i];
            }
        } 
        //fprintf(stdout, "OctreeNode deleted\n");
    }

};


class OctreeLeaf : public  OctreeBaseNode
{
    public:
    OctreeLeaf(const BoundingBox3f& bbox, std::vector<int>& triangle_idx)
    {
        m_bbox = bbox;
        m_triangle_idx = triangle_idx;
    }
    ~OctreeLeaf()
    {
        //fprintf(stdout, "OctreeLeaf deleted\n");
        for(size_t i = 0; i < 8; ++i)
        {
            if(this->children[i])
            {
                fprintf(stdout, "WTF\n");
            }
        }
    }

    //std::vector<int> triangle_idx;
};

NORI_NAMESPACE_END
