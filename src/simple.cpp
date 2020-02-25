#include <nori/integrator.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

class SimpleIntegrator : public Integrator
{
    public:
    SimpleIntegrator(const PropertyList& prop):
    myPoint(prop.getPoint("position")), 
    myColor(prop.getColor("energy"))
    {
        fprintf(stdout, "position %f %f %f;\nenergy %f %f %f;\n", myPoint.x(), myPoint.y(), myPoint.z(), myColor.x(), myColor.y(), myColor.z());
    }

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const
    {
        Intersection its;
        if(!scene->rayIntersect(ray, its))
        {
            return Color3f(0, 0, 0);
        }
        Vector3f normal = its.shFrame.n;
        Point3f p = this->myPoint;
        Point3f x = its.p;
        Point3f diff = p - x;
        float norm_diff = diff.x() * diff.x() + diff.y() * diff.y() + diff.z() * diff.z();
        diff = diff / std::sqrt(norm_diff);
        float cos = (normal.x() * diff.x() + normal.y() * diff.y() + normal.z() * diff.z());
        Ray3f vis(x, diff);
        if(scene->rayIntersect(vis))
        {
            return Color3f(0, 0, 0);
        }
        float tmp = 1.0 / (4.0 * M_PI * M_PI) * std::max(0.0f , cos) / (norm_diff);
        return Color3f(this->myColor.x() * tmp, this->myColor.y() * tmp, this->myColor.z() * tmp);
    }

    std::string toString() const
    {
        return "SimpleIntegrator[]";
    }
    protected:
    std::string m_myproperty;
    Point3f myPoint;
    Color3f myColor;
};

NORI_REGISTER_CLASS(SimpleIntegrator, "simple");
NORI_NAMESPACE_END