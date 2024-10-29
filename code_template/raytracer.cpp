#include <iostream>
#include <math.h>
#include "parser.h"
#include "ppm.h"

using namespace parser;
typedef unsigned char RGB[3];

/*
 * for each pixel s
 * compute r // done
 * make t_min inf, obj null // done
 *      foreach obj o
 *          if r intersects o
 *              if t < t_min:
 *                  t_min = t, obj = o
 *      if obj not null
 *      pixel_color = L_a
 *      foreach light l:
 *          compute shadow ray s (from x to l)
 *          foreach object p
 *              if s intersects p before the light source
 *                  continue // in shadow!
 *          pixel_color += L_d + L_s
 *      else pixel_color = bg_color
 *
 * */

struct Ray{
    Vec3f origin;
    Vec3f direction;

    Ray(){}
    Ray(Vec3f org, Vec3f dir): origin(org), direction(dir){}

};

struct Intersection {
    int materialId;
    int objId;
    float tValue;
    bool intersects;
    Vec3f surfaceNormal;
    Vec3f intPoint;
    Vec3f color;

    Intersection():intersects(false), tValue(MAXFLOAT){
        color.x = -1;
        color.y = -1;
        color.z = -1;
    }

    Intersection(bool itsct): intersects(itsct){}

};


float calcDeterminant(Vec3f &v0, Vec3f &v1, Vec3f &v2){
    return v0.x*(v1.y*v2.z - v2.y*v1.z) - v0.y*( v1.x*v2.z - v1.z*v2.x) + v0.z*(v1.x*v2.y-v1.y*v2.x);
}

float dotProduct(Vec3f &v1, Vec3f &v2){
    return (v1.x*v2.x + v1.y*v2.y + v1.z*v2.z);
}

Vec3f normalize(Vec3f vec){
    Vec3f newVec;
    float len = sqrt(dotProduct(vec, vec));
    newVec.x = vec.x/len;
    newVec.y = vec.y/len;
    newVec.z = vec.z/len;
    return newVec;
}

Vec3f crossProduct(Vec3f &v1, Vec3f &v2){
    Vec3f result;
    result.x = v1.y*v2.z - v1.z*v2.y;
    result.y = v1.z*v2.x - v1.x*v2.z;
    result.z = v1.x*v2.y - v1.y*v2.x;

    return result;
}


float calcDistance(Vec3f &v1, Vec3f &v2){
    return sqrt(pow((v1.x - v2.x),2) + pow((v1.y - v2.y),2) + pow((v1.z - v2.z),2));
}

Vec3f clamp(Vec3f color){
    if (color.x > 255){
        color.x = 255;
    }

    if(color.y > 255){
        color.y = 255;
    }

    if(color.z > 255){
        color.z = 255;
    }

    return Vec3f(round(color.x), round(color.y), round(color.z));

}


Ray getRay(int w, int h, Camera &cam){

    float l = cam.near_plane.x;
    float r = cam.near_plane.y;
    float b = cam.near_plane.z;
    float t = cam.near_plane.w;
    float s_u = (r - l)*(w+0.5)/cam.image_width;
    float s_v = (t- b)*(h+0.5)/cam.image_height;

    Vec3f u = normalize(crossProduct(cam.gaze, cam.up));
    Vec3f v = normalize(crossProduct(u, cam.gaze));

    Vec3f m = cam.position+(cam.gaze*cam.near_distance);
    Vec3f q = m + (u*l) + (v*t);
    Vec3f s = q + (u*s_u) - (v*s_v);

    Ray ray;
    ray.direction = normalize((s-cam.position));
    ray.origin = cam.position;
    return ray;
}

Vec3f intersectionPt(Ray &ray, double t){
    // r(t) = o +td
    Vec3f p = ray.origin + ray.direction*t;
    /*p.x = ray.origin.x + ray.direction.x * t;
    p.y = ray.origin.y * ray.direction.y;
    p.z = ray.origin.z * ray.direction.z;*/

    return p;
}

/*
 *
 * Intersections
 *
 * */

Intersection sphereIntersection(Ray &ray, Scene &scene, Sphere &sphere){
    Intersection pt;
    Vec3f center = scene.vertex_data[sphere.center_vertex_id - 1];
    Vec3f o_c = (ray.origin-center);
    Vec3f o_cDoubled= (ray.origin-center)*2.0;

    float a = dotProduct(ray.direction, ray.direction);
    float b = dotProduct(ray.direction, o_cDoubled);
    float c = dotProduct(o_c, o_c) - pow(sphere.radius, 2);

    float discriminant = (b*b) - 4*(a*c);

    if (discriminant >= 0){
        float firstTerm = (-1)*b/2;
        float secondTerm = sqrt(pow(b/2, 2) - a*(dotProduct(o_c, o_c))- pow(sphere.radius,2));

        float t1 = (firstTerm + secondTerm)/a;
        float t2 = (firstTerm - secondTerm)/a;

        float t = (t1 > t2) ? t2: t1;
        if(t > 0){
            pt.intersects = true;
            pt.intPoint = ray.origin + (ray.direction*t);
            pt.surfaceNormal = normalize((pt.intPoint-center)/sphere.radius);
            pt.materialId = sphere.material_id;
            pt.tValue = t;
        }

    }else{
        pt.intersects = false;
    }

    return pt;
}

Intersection triangleIntersection(Ray &ray, Scene &scene, Triangle &triangle){

    Intersection pt;

    Vec3f &a = scene.vertex_data[triangle.indices.v0_id -1];
    Vec3f &b = scene.vertex_data[triangle.indices.v1_id -1];
    Vec3f &c = scene.vertex_data[triangle.indices.v2_id -1];

    Vec3f ab = a - b;
    Vec3f ac = a - c;
    Vec3f ao = a - ray.origin;

    float beta_determinant = calcDeterminant(ao, ac, ray.direction);
    float gamma_determinant = calcDeterminant(ab, ao, ray.direction);
    float t_determinant = calcDeterminant(ab, ac, ao);

    float a_determinant = calcDeterminant(ab, ac, ray.direction);

    if (a_determinant == 0){
        return pt;
    }

    float t = t_determinant/a_determinant;
    if (t <= 0){
        return pt;
    }

    float beta = beta_determinant/a_determinant;
    if(beta < 0 || beta > 1){
        return pt;
    }

    float gamma = gamma_determinant/a_determinant;
    if (gamma < 0 || gamma > (1-gamma)){
        return pt;
    }

    Vec3f a_b = b - a;
    Vec3f a_c = c - a;

    pt.intersects = true;
    pt.intPoint = ray.origin+(ray.direction*t);
    pt.surfaceNormal = normalize(crossProduct(a_c, a_b)); // bu dogru mu bak!!!!
    pt.tValue = t;
    pt.materialId = triangle.material_id;

    return pt;
}

Intersection meshIntersection(Ray &ray, Scene &scene, Mesh &mesh){
    Intersection closest;
    for (const Face &face: mesh.faces) {
        Triangle triTemp(mesh.material_id, face);
        Intersection temp = triangleIntersection(ray, scene, triTemp);
        if (temp.intersects && temp.tValue < closest.tValue){
            closest.tValue = temp.tValue;
            closest.intersects = true;
            closest.surfaceNormal = closest.surfaceNormal;
        }
    }

    closest.intPoint = intersectionPt(ray, closest.tValue);
    return closest;
}

/*
 *
 * shading
 *
 * */

Vec3f irradiance(PointLight &light, Vec3f &point){
    Vec3f i = light.position - point;
    float d = dotProduct(i, i);
    if(d == 0.0){
        return Vec3f();
    } else{
        return light.intensity/d;
    }
}

Vec3f diffuseShading(PointLight &light, Scene &scene, Intersection &intersection){
    Vec3f irrd = irradiance(light, intersection.intPoint);
    Vec3f l = normalize((light.position - intersection.intPoint));
    float  p = dotProduct(l, intersection.surfaceNormal);
    if (p < 0){
        p = 0;
    }
    return scene.materials[intersection.materialId-1].diffuse*(irrd*p);

}

Vec3f specularShading(PointLight &light, Scene &scene, Intersection &intersection, Ray &ray){
    Vec3f irrd = irradiance(light, intersection.intPoint);
    Material material = scene.materials[intersection.materialId-1];
    Vec3f l = normalize(light.position - intersection.intPoint);
    Vec3f r = normalize(l - ray.direction);

    float p = dotProduct(intersection.surfaceNormal, r);

    if (p < 0){
        p = 0;
    }

    return material.specular * irrd * pow(p, material.phong_exponent);
}

/*
 *
 * main handler
 *
 */

Intersection hitFunction(Ray &ray, Scene &scene, Camera &cam, int mirrorRecDepth){

    Intersection result;

    if (mirrorRecDepth > scene.max_recursion_depth){
        return result;
    }

    for (int i = 0; i < scene.spheres.size(); ++i) {
        Intersection closestSphere = sphereIntersection(ray, scene, scene.spheres[i]);
        if (closestSphere.intersects && closestSphere.tValue < result.tValue){
            result = closestSphere;
        }
    }

    for (int i = 0; i < scene.triangles.size(); ++i) {
        Intersection closestTriangle = triangleIntersection(ray,scene,scene.triangles[i]);
        if (closestTriangle.intersects && closestTriangle.tValue < result.tValue){
            result = closestTriangle;
        }
    }

    for (int i = 0; i < scene.meshes.size(); ++i) {
        Intersection closestMesh = meshIntersection(ray, scene, scene.meshes[i]);
        if (closestMesh.intersects && closestMesh.tValue < result.tValue){
            result = closestMesh;
        }
    }

    if(result.intersects){

        //check for shadow

        Material material = scene.materials[result.materialId-1];
        Vec3f color = material.ambient * scene.ambient_light;

        for (PointLight &light : scene.point_lights){
            bool inShadow = false;
            float distance = calcDistance(light.position, cam.position);

            Vec3f w = normalize(light.position - result.intPoint);
            Vec3f e = w * scene.shadow_ray_epsilon;

            Ray shadowRay(result.intPoint + e, w);

            //t value for point lights, if it is lower than shadow's, object is in light, otherwise in shadow.
            float tLight = (light.position - shadowRay.origin).x/shadowRay.direction.x;

            for (int i = 0; i < scene.spheres.size(); ++i) {
                Sphere &sphere = scene.spheres[i];
                Intersection sphInts = sphereIntersection(shadowRay, scene, sphere);
                if(sphInts.intersects && sphInts.tValue < tLight){
                    inShadow = true;
                }
            }

            for (int i = 0; i < scene.triangles.size(); ++i) {
                Triangle &triangle = scene.triangles[i];
                Intersection triInts = triangleIntersection(shadowRay, scene, triangle);
                if (triInts.intersects && triInts.tValue < tLight){
                    inShadow = true;
                }
            }

            if(!inShadow){
                for (int i = 0; i < scene.meshes.size(); ++i) {
                    Mesh &mesh = scene.meshes[i];
                    Intersection meshInts = meshIntersection(shadowRay, scene, mesh);
                    if (meshInts.intersects && meshInts.tValue < tLight){
                        inShadow = true;
                    }

                }
            }

            if(!inShadow || (inShadow && distance == 0)){
                Vec3f diffuse = diffuseShading(light, scene, result);
                Vec3f specular = specularShading(light, scene, result, ray);
                color = color + diffuse + specular;
                result.color = color;

            }
        }

        if(material.is_mirror){
            Vec3f reflectionDirection = normalize((ray.direction - result.surfaceNormal) * (dotProduct(ray.direction, result.surfaceNormal) * 2.0));
            Vec3f eps = reflectionDirection * scene.shadow_ray_epsilon;
            Ray reflection(result.intPoint+eps, reflectionDirection);
            Intersection reflectionIntersection= hitFunction(reflection, scene, cam, mirrorRecDepth + 1);

            if (reflectionIntersection.intersects){
                color = color + scene.materials[reflectionIntersection.materialId-1].mirror;
            }
        }
        result.color = color;
    }

    return result;

}


int main(int argc, char* argv[]) {
    // Sample usage for reading an XML scene file
    parser::Scene scene;
    scene.loadFromXml(argv[1]);

    for (Camera &cam: scene.cameras) {
        const int width = cam.image_width, height = cam.image_height;
        unsigned char *image = new unsigned char[width * height * 3];

        int i = 0;

        for (int h = 0; h < width; h++) {
            for (int w = 0; w < height; w++) {
                Ray ray = getRay(w, h, cam);
                Intersection intersection = hitFunction(ray, scene, cam, scene.max_recursion_depth);
                //find intersection
                //calculate rgb, pls clamp.

                if (intersection.intersects) {
                    clamp(intersection.color);
                    image[i++] = intersection.color.x;
                    image[i++] = intersection.color.y;
                    image[i++] = intersection.color.z;
                } else {
                    image[i++] = scene.background_color.x;
                    image[i++] = scene.background_color.y;
                    image[i++] = scene.background_color.z;
                }
            }
        }
        write_ppm("test.ppm", image, width, height);
    }
    return 0;
}


