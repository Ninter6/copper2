#include <algorithm>
#include <iostream>
#include <fstream>
#include <vector>
#include <ranges>
#include <random>
#include <array>

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

    int sign_x() const { return A.x > B.x ? -1 : 1; }
    int sign_y() const { return A.y > B.y ? -1 : 1; }

    auto view() const {
        return line_view(A.x, A.y, B.x, B.y);
    }
};

auto line_steps_x_view(const Line& l, int prefer) {
    constexpr auto always_true = [](auto&&) { return true; };
    return l.view() | std::views::chunk_by([](auto&& p1, auto&& p2) {
        return p1.y == p2.y;
    }) | std::views::transform([=, f = l.sign_x() == prefer](auto&& c) {
        return f ? c.front() : std::ranges::find_last_if(c,
            always_true).front();
    });
}

auto trapezoid_view(const Line& l, const Line& r) {
    auto le = line_steps_x_view(l, 1);
    auto re = line_steps_x_view(r,-1);
    return std::views::zip(le, re);
}

struct Trapezoid {
    Line L, R;

    Trapezoid(const Line& l, const Line& r) : L(l), R(r) {
        check_lr_and_swap();
    }

    void check_lr_and_swap() {
        if (L.A.x > R.A.x)
            std::swap(L, R);
        else if (L.B.x > R.B.x)
            std::swap(L, R);
    }

    auto view() const {
        return trapezoid_view(L, R);
    }
};

auto scanline_view(const Trapezoid& trap) {
    return trap.view() | std::views::transform([](auto&& a) {
        auto&& [f, l] = a;
        return std::views::iota(f.x, l.x + 1)
            | std::views::transform([y=f.y](auto&& x) {
                return m3::ivec2{x, y};
            });
    });
}

void draw_triangle(cu::ImageRGB& img, std::array<m3::ivec2, 3> va, const m3::vec3& col) {
    std::ranges::sort(va, [](auto&& a, auto&& b) {
        return a.y < b.y;
    });

    if (va[0].y == va[2].y) {
        auto [mn, mx] = std::ranges::minmax_element(va, [](auto&& a, auto&& b) {
            return a.x < b.x;
        });
        for (int x : std::views::iota(mn->x, mx->x +1))
            img[x, va[0].y] = col;
        return;
    }

    if (va[0].y == va[1].y) {
        Trapezoid trap{
            {va[0], va[2]},
            {va[1], va[2]}
        };
        for (auto&& v : scanline_view(trap) | std::views::join)
            img[v.x, v.y] = col;
        return;
    }
    if (va[1].y == va[2].y) {
        Trapezoid trap{
                {va[0], va[1]},
                {va[0], va[2]}
        };
        for (auto&& v : scanline_view(trap) | std::views::join)
            img[v.x, v.y] = col;
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
    for (auto&& v : scanline_view(t1) | std::views::join)
        img[v.x, v.y] = col;
    for (auto&& v : scanline_view(t2) | std::views::join)
        img[v.x, v.y] = col;
}

void draw_line(cu::ImageRGB& img, int fx, int fy, int lx, int ly, const m3::vec3& col) {
    if (fy == ly)
        for (int x : std::views::iota(fx, lx + 1))
            img.set_or_ignore(x, fy, col);
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
    do {draw_line(img, cx-dx, cy-dy, cx+dx, cy-dy, col);
        draw_line(img, cx-dy, cy-dx, cx+dy, cy-dx, col);
        draw_line(img, cx-dy, cy+dx, cx+dy, cy+dx, col);
        draw_line(img, cx-dx, cy+dy, cx+dx, cy+dy, col);
        e -= (e += ++dx<<1) > dy ? dy--<<1 : 1;
    } while (dx <= dy);
}

int main() {
    std::cout << "Hello, World!" << std::endl;

    cu::ImageRGB img{50, 50};

    for (auto&& [p, x, y] : img.view() | std::views::filter([](auto&& a) {
        auto&& [_, x, y] = a;
        return std::abs((int)(x - y)) > 30;
    })) {
        p = {0, (float)x / img.width, (float)y / img.height};
    }

    draw_circle_line(img, 25, 25, 15, {1, 0, 0});

    // for (auto&& [p, x, y] : img.view() | std::views::filter([](auto&& a) {
    //     auto&& [_, x, y] = a;
    //     return x == 30;
    // })) {
    //     p = {0, 0, 1};
    // }

    // Line l{{10, 10}, {30, 25}};
    // Line r{{30, 10}, {30, 25}};
    //
    // Trapezoid trap{l, r};
    //
    // for (auto&& [x, y] : scanline_view(trap) | std::views::join) {
    //     img[x, y] = {0, 1 ,0};
    // }

    // std::array<m3::ivec2, 3> va;
    //
    // std::mt19937 e{std::random_device{}()};
    // std::ranges::generate(va, [&] {
    //     return m3::ivec2(m3::rand_unit_vec2(e) * 250 + 250);
    // });
    //
    // draw_triangle(img, va, {1, 0, 0});
    //
    // for (auto&& p : va)
    //     draw_circle(img, p.x, p.y, 5, {0, 1, 0});

    cu::export_ppm(img, FILE_ROOT"img.ppm");

    return 0;
}
