#include "common.hpp"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr double pi = 3.14159265358979323846;
constexpr std::size_t trail_length = 600;

class Glfw {
public:
    Glfw() { if (!glfwInit()) throw std::runtime_error("Cannot initialize GLFW"); }
    ~Glfw() { glfwTerminate(); }
    Glfw(const Glfw&) = delete;
    Glfw& operator=(const Glfw&) = delete;
};

void point(const Eigen::Vector3d& p, double radius) {
    glBegin(GL_TRIANGLE_FAN);
    glVertex2d(p.x(), p.y());
    for (int i = 0; i <= 32; ++i) {
        const double angle = 2.0 * pi * i / 32.0;
        glVertex2d(p.x() + radius * std::cos(angle), p.y() + radius * std::sin(angle));
    }
    glEnd();
}

void line(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    glVertex2d(a.x(), a.y());
    glVertex2d(b.x(), b.y());
}

// Supports drawn as small markers: pivots, anchors, and prescribed particles.
std::vector<Eigen::Vector3d> supports(const physics::Model& model, double time) {
    std::vector<Eigen::Vector3d> result;
    for (const auto& c : model.distance_constraints) result.push_back(c.pivot.position(time));
    for (const auto& f : model.harmonic_forces) result.push_back(f.equilibrium.position(time));
    return result;
}

// Half-height of the view: grows to keep everything visible, never shrinks.
double extent(const physics::Model& model, const physics::State& state, double time) {
    double result = 0.0;
    for (const auto& p : state.positions) result = std::max(result, std::max(std::abs(p.x()), std::abs(p.y())));
    for (const auto& p : supports(model, time)) result = std::max(result, std::max(std::abs(p.x()), std::abs(p.y())));
    return result;
}

// Rendering consumes const configuration/state; it never modifies the physics.
// The view is the x-y plane.
void render(GLFWwindow* window, const physics::Model& model, const physics::State& state, double time,
            const std::vector<std::deque<Eigen::Vector3d>>& trails, double scale) {
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    if (width == 0 || height == 0) return;
    const double aspect = static_cast<double>(width) / height;
    glViewport(0, 0, width, height);
    glClearColor(0.07f, 0.08f, 0.10f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-scale * aspect, scale * aspect, -scale, scale, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glLineWidth(1.0f);
    glColor3f(0.25f, 0.35f, 0.45f);
    for (const auto& trail : trails) {
        glBegin(GL_LINE_STRIP);
        for (const auto& p : trail) glVertex2d(p.x(), p.y());
        glEnd();
    }
    glLineWidth(2.0f);
    glColor3f(0.5f, 0.7f, 0.8f); // rigid rods
    glBegin(GL_LINES);
    for (const auto& c : model.distance_constraints) line(c.pivot.position(time), state.positions[c.particle]);
    for (const auto& l : model.links) line(state.positions[l.first], state.positions[l.second]);
    glEnd();
    glColor3f(0.6f, 0.8f, 0.4f); // springs
    glBegin(GL_LINES);
    for (const auto& f : model.harmonic_forces) line(f.equilibrium.position(time), state.positions[f.particle]);
    for (const auto& s : model.springs) line(state.positions[s.first], state.positions[s.second]);
    glEnd();
    glColor3f(0.9f, 0.6f, 0.2f);
    for (const auto& p : supports(model, time)) point(p, scale * 0.02);
    for (const auto& p : model.prescribed) point(state.positions[p.particle], scale * 0.03);
    glColor3f(0.3f, 0.8f, 0.9f);
    std::vector<bool> prescribed(state.positions.size(), false);
    for (const auto& p : model.prescribed) prescribed[p.particle] = true;
    for (std::size_t i = 0; i < state.positions.size(); ++i) {
        if (!prescribed[i]) point(state.positions[i], scale * 0.035);
    }
    glfwSwapBuffers(window);
}
} // namespace

int main(int argc, char** argv) {
    try {
        const auto options = physics::app::parseOptions(argc, argv);
        if (options.help) { std::cout << physics::app::usage(argv[0]); return 0; }
        if (options.list) { std::cout << physics::app::exampleList(); return 0; }
        physics::app::Run run(options);
        Glfw glfw;
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
        std::unique_ptr<GLFWwindow, decltype(&glfwDestroyWindow)> window(
            glfwCreateWindow(1000, 700, "Physics Engine", nullptr, nullptr), glfwDestroyWindow);
        if (!window) throw std::runtime_error("Cannot create an OpenGL window");
        glfwMakeContextCurrent(window.get());
        glfwSwapInterval(1);
        const auto& sim = run.simulation();
        double scale = std::max(1.0, 1.3 * extent(sim.model(), sim.state(), sim.time()));
        std::vector<std::deque<Eigen::Vector3d>> trails(sim.state().positions.size());
        double previous = glfwGetTime();
        double accumulator = 0.0;
        while (!glfwWindowShouldClose(window.get())) {
            glfwPollEvents();
            if (glfwGetKey(window.get(), GLFW_KEY_ESCAPE) == GLFW_PRESS) break;
            const double now = glfwGetTime();
            accumulator += std::min(now - previous, 0.25);
            previous = now;
            // Bound per-frame work for responsiveness. Slow machines run behind
            // wall time; they never change the configured physics step.
            for (int work = 0; work < 1000 && !run.finished(); ++work) {
                const double dt = std::min(options.dt, options.duration - sim.time());
                if (accumulator < dt) break;
                run.advance();
                accumulator -= dt;
                for (std::size_t i = 0; i < trails.size(); ++i) {
                    trails[i].push_back(sim.state().positions[i]);
                    if (trails[i].size() > trail_length) trails[i].pop_front();
                }
            }
            scale = std::max(scale, 1.1 * extent(sim.model(), sim.state(), sim.time()));
            const std::string title = run.summary() + " | Esc to close";
            glfwSetWindowTitle(window.get(), title.c_str());
            render(window.get(), sim.model(), sim.state(), sim.time(), trails, scale);
            if (run.finished()) break;
        }
        run.finishOutput();
        std::cout << run.summary() << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}
