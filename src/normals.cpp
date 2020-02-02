#include <nori/integrator.h>
NORI_NAMESPACE_BEGIN
class NormalIntegrator : public Integrator {
    public:
    NormalIntegrator(const PropertyList& props)
    {
        m_myproperty = props.getString("myProperty");
        std::cout << "Para values = " << m_myproperty << "\n";
    }

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const
    {
        return Color3f(0, 1, 0);
    }
    std::string toString() const
    {
        return tfm::format(
            "NormalIntegrator[\n"
            " myProperty = \"%s\"\n"
            "]",
            m_myproperty
        );
    }
    protected:
    std::string m_myproperty;
};

NORI_REGISTER_CLASS(NormalIntegrator, "normals");
NORI_NAMESPACE_END

