#include <algorithm>
#include <iostream>
#include <fstream>
#include <cassert>
#include <vector>
#include <ranges>
#include <random>
#include <array>

#include "timer.hh"

import core;
import image;
import util;

struct line_phase {
    int x, y;
    int err;
    int dx, dy;
    int sx, sy;
};

line_phase line_bresenham(line_phase p) {
    int e2 = p.err << 1;
    if (e2 > -p.dy) {
        p.err -= p.dy;
        p.x += p.sx;
    }
    if (e2 < p.dx) {
        p.err += p.dx;
        p.y += p.sy;
    }
    return p;
}

auto line_view(int fx, int fy, int lx, int ly) {
    line_phase p0{};
    p0.x = fx; p0.y = fy;
    p0.dx = std::abs(lx - fx);
    p0.dy = std::abs(ly - fy);
    p0.sx = fx > lx ? -1 : 1;
    p0.sy = fy > ly ? -1 : 1;
    p0.err = p0.dx - p0.dy;

    return cu::util::iterate(p0, line_bresenham)
        | std::views::take(std::max(p0.dx, p0.dy) + 1);
}

struct Line {
    m3::ivec2 A, B;

    void flip() { std::swap(A, B); }

    [[nodiscard]] int sign_x() const { return A.x > B.x ? -1 : 1; }
    [[nodiscard]] int sign_y() const { return A.y > B.y ? -1 : 1; }

    [[nodiscard]] auto view() const {
        return line_view(A.x, A.y, B.x, B.y);
    }
};

struct step_phase {
    int x, y;
    int l, r;
    int err;
    int lev;
    int lp, rp;
    int dx, dy;
    int sx, sy;
};

step_phase line_step_y(step_phase p) {
    p.y += p.sy;
    p.x = p.r + p.lp;
    p.err += p.dx << 1;
    if (p.err >= p.dy) {
        p.x += p.sx;
        p.err -= p.dy << 1;
    }
    p.l = p.lp ? p.r + p.sx : p.x;
    p.r = p.x;
    if (++p.lev < p.dy)
        p.r += p.rp;
    return p;
}

auto line_step_y_view(const Line& l, int prefer) {
    step_phase p0{};
    p0.x = l.A.x; p0.y = l.A.y;
    p0.dx = std::abs(l.B.x - l.A.x);
    p0.dy = std::abs(l.B.y - l.A.y);
    p0.sx = l.A.x > l.B.x ? -1 : 1;
    p0.sy = l.A.y > l.B.y ? -1 : 1;

    int pad = p0.dx / std::max(p0.dy, 1);
    p0.dx = p0.dx % std::max(p0.dy, 1);
    p0.lp = pad ? p0.sx * ((pad >> 1) + 1) : 0;
    p0.rp = pad ? p0.sx * ((pad - 1) >> 1) : 0;

    p0.l = p0.x;
    if (!p0.dy)
        p0.x += p0.lp;
    p0.r = p0.x + p0.rp;

    return cu::util::iterate(p0, line_step_y)
        | std::views::take_while([](auto&& p) { return p.lev <= p.dy; })
        | std::views::transform([=](auto&& p) -> m3::ivec2 {
            switch (prefer) {
                case -1: return {p.r, p.y}; // last
                case  1: return {p.l, p.y}; // first
                case  0: return {p.x, p.y}; // mid
                default: assert(false);
            }
        });
}

auto trapezoid_view(const Line& l, const Line& r) {
    auto le = line_step_y_view(l, l.sign_x());
    auto re = line_step_y_view(r,-l.sign_x());
    return std::views::zip(le, re);
}

struct Trapezoid {
    Line L, R;

    Trapezoid(const Line& l, const Line& r) : L(l), R(r) {
        check_lr_and_swap();
    }

    void check_lr_and_swap() {
        if (L.A.x > R.A.x || L.B.x > R.B.x)
            std::swap(L, R);
    }

    [[nodiscard]] auto view() const {
        return trapezoid_view(L, R);
    }
};

void draw_horiz_line(cu::ImageRGB& img, int fx, int lx, int y, const m3::vec3& col) {
    for (int x : std::views::iota(fx, lx + 1))
        img.set_or_ignore(x, y, col);
}

void draw_trapezoid(cu::ImageRGB& img, const Trapezoid& t, const m3::vec3& col) {
    for (auto&& [l, r] : t.view())
        draw_horiz_line(img, l.x, r.x, l.y, col);
}

void draw_triangle(cu::ImageRGB& img, std::array<m3::ivec2, 3> va, const m3::vec3& col) {
    std::ranges::sort(va, [](auto&& a, auto&& b) {
        return a.y < b.y;
    });

    if (va[0].y == va[2].y) {
        auto [mn, mx] = std::ranges::minmax_element(va,
            [](auto&& a, auto&& b) { return a.x < b.x; });
        draw_horiz_line(img, mn->x, mx->x, va[0].y, col);
        return;
    }

    if (va[0].y == va[1].y) {
        Trapezoid trap{
            {va[0], va[2]},
            {va[1], va[2]}
        };
        draw_trapezoid(img, trap, col);
        return;
    }
    if (va[1].y == va[2].y) {
        Trapezoid trap{
                {va[0], va[1]},
                {va[0], va[2]}
        };
        draw_trapezoid(img, trap, col);
        return;
    }

    int dx = va[2].x - va[0].x;
    int dy0 = va[2].y - va[0].y;
    int dy1 = va[1].y - va[0].y;
    int xm = va[0].x + (dx*dy1+(dy0>>1))/dy0;
    m3::ivec2 M{xm, va[1].y};

    Trapezoid t1{
        {va[0], M},
        {va[0], va[1]}
    }, t2{
        {M, va[2]},
        {va[1], va[2]}
    };
    draw_trapezoid(img, t1, col);
    draw_trapezoid(img, t2, col);
}

void draw_line(cu::ImageRGB& img, int fx, int fy, int lx, int ly, const m3::vec3& col) {
    if (fy == ly)
        draw_horiz_line(img, fx, lx, fy, col);
    else
        for (auto&& p : line_view(fx, fy, lx, ly))
            img.set_or_ignore(p.x, p.y, col);
}

void draw_circle_line(cu::ImageRGB& img, int cx, int cy, int r, const m3::vec3& col) {
    int dx = 0, dy = r, e = 0;
    do {img.at(cx+dx, cy+dy) = col;
        img.at(cx+dx, cy-dy) = col;
        img.at(cx-dx, cy+dy) = col;
        img.at(cx-dx, cy-dy) = col;
        img.at(cx+dy, cy+dx) = col;
        img.at(cx+dy, cy-dx) = col;
        img.at(cx-dy, cy+dx) = col;
        img.at(cx-dy, cy-dx) = col;
        e -= (e += ++dx<<1) > dy ? dy--<<1 : 1;
    } while (dx <= dy);

    // do {img[cx+dx, cy-1+dy] = col;
    //     img[cx+dx, cy-dy] = col;
    //     img[cx-1-dx, cy-1+dy] = col;
    //     img[cx-1-dx, cy-dy] = col;
    //     img[cx-1+dy, cy+dx] = col;
    //     img[cx-1+dy, cy-1-dx] = col;
    //     img[cx-dy, cy+dx] = col;
    //     img[cx-dy, cy-1-dx] = col;
    //     e -= (e += ++dx<<1) < dy ? 0 : --dy<<1;
    // } while (dx < dy);
}

void draw_circle(cu::ImageRGB& img, int cx, int cy, int r, const m3::vec3& col) {
    int dx = 0, dy = r, e = 0;
    do {draw_horiz_line(img, cx-dx, cx+dx, cy-dy, col);
        draw_horiz_line(img, cx-dy, cx+dy, cy-dx, col);
        draw_horiz_line(img, cx-dy, cx+dy, cy+dx, col);
        draw_horiz_line(img, cx-dx, cx+dx, cy+dy, col);
        e -= (e += ++dx<<1) > dy ? dy--<<1 : 1;
    } while (dx <= dy);
}

int main() {
    std::cout << "Hello, World!" << std::endl;

    cu::ImageRGB img{500, 500};
    img.fill(m3::vec3{.67f});

    // for (auto&& [p, x, y] : img.view()) {
    //     if (std::abs((int)(x - y)) > 30)
    //         p = {0, (float)x / img.width, (float)y / img.height};
    // }

    // draw_circle_line(img, 25, 25, 15, {1, 0, 0});

    // for (auto&& [p, x, y] : img.view() | std::views::filter([](auto&& a) {
    //     auto&& [_, x, y] = a;
    //     return x == 30;
    // })) {
    //     p = {0, 0, 1};
    // }

    std::array<m3::ivec2, 3> va;

    va[0] = {10, 10};
    va[1] = {10, 490};
    va[2] = {490, 250};
    // std::mt19937 e{std::random_device{}()};
    // std::ranges::generate(va, [&] {
    //     return m3::ivec2(m3::rand_unit_vec2(e) * 250 + 250);
    // });

    TICK();
    draw_triangle(img, va, {1, 0, 0});

    for (auto&& p : va)
        draw_circle(img, p.x, p.y, 5, {0, 1, 0});
    TOCK();

    cu::Canvas can{&img, 0, 200, 500, 100};
    can.flip_x();

    TICK();
    cu::export_ppm(img, FILE_ROOT"img.ppm");
    TOCK();

    return 0;
}
