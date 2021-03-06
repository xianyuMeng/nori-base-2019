#include <nori/integrator.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN
class NormalIntegrator : public Integrator {
    public:
    NormalIntegrator(const PropertyList& props)
    {
//        m_myproperty = props.getString("myProperty");
//        std::cout << "Para values = " << m_myproperty << "\n";
    }

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const
    {
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);

        /* Return the component-wise absolute
           value of the shading normal as a color */
        Normal3f n = its.shFrame.n.cwiseAbs();
        return Color3f(n.x(), n.y(), n.z());
    }
    std::string toString() const
    {
//        return tfm::format(
//            "NormalIntegrator[\n"
//            " myProperty = \"%s\"\n"
//            "]",
//            m_myproperty
//        );
		return "NormalIntegrator[]";
    }
    protected:
    std::string m_myproperty;
};

NORI_REGISTER_CLASS(NormalIntegrator, "normals");
NORI_NAMESPACE_END

