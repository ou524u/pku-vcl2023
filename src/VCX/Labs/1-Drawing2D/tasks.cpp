#include <random>

#include <spdlog/spdlog.h>

#include "Labs/1-Drawing2D/tasks.h"

using VCX::Labs::Common::ImageRGB;

namespace VCX::Labs::Drawing2D {
/******************* 1.Image Dithering *****************/
void DitheringThreshold(
    ImageRGB& output,
    ImageRGB const& input) {
    for (std::size_t x = 0; x < input.GetSizeX(); ++x)
        for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
            glm::vec3 color = input.At(x, y);
            output.At(x, y) = {
                color.r > 0.5 ? 1 : 0,
                color.g > 0.5 ? 1 : 0,
                color.b > 0.5 ? 1 : 0,
            };
        }
}

// Function to generate a random uniform number in [-0.05, 0.5]
float GenerateRandomUniform(const float& fa, const float& fb) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<float> dist(fa, fb);
    return dist(gen);
}

void DitheringRandomUniform(
    ImageRGB& output,
    ImageRGB const& input) {
    for (std::size_t x = 0; x < input.GetSizeX(); ++x)
        for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
            glm::vec3 color = input.At(x, y);

            // Add random uniform noise to each channel
            float uni_rand = GenerateRandomUniform(-0.5, 0.5);
            color.r += uni_rand;
            color.g += uni_rand;
            color.b += uni_rand;

            // Perform dithering
            output.At(x, y) = {
                color.r > 0.5f ? 1 : 0,
                color.g > 0.5f ? 1 : 0,
                color.b > 0.5f ? 1 : 0,
            };
        }
}

glm::vec3 GetImageRGBAverage(ImageRGB const& input) {
    float fr = 0;
    float fg = 0;
    float fb = 0;
    for (std::size_t x = 0; x < input.GetSizeX(); ++x)
        for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
            glm::vec3 color = input.At(x, y);
            fr += color.r;
            fg += color.g;
            fb += color.b;
        }
    std::size_t input_size = input.GetSizeX() * input.GetSizeY();
    fr /= input_size;
    fg /= input_size;
    fb /= input_size;
    return {fr, fg, fb};
}

void DitheringRandomBlueNoise(
    ImageRGB& output,
    ImageRGB const& input,
    ImageRGB const& noise) {
    // your code here:
    glm::vec3 noise_average = GetImageRGBAverage(noise);
    for (std::size_t x = 0; x < input.GetSizeX(); ++x)
        for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
            glm::vec3 color = input.At(x, y);
            glm::vec3 noise_color = noise.At(x, y);
            color.r += noise_color.r - noise_average.r;
            color.g += noise_color.g - noise_average.g;
            color.b += noise_color.b - noise_average.b;
            output.At(x, y) = {
                color.r > 0.5 ? 1 : 0,
                color.g > 0.5 ? 1 : 0,
                color.b > 0.5 ? 1 : 0,
            };
        }
}

// Define the 3x3 dithering matrix
constexpr int ditherMatrix[3][3] = {
    {6, 8, 4},
    {1, 9, 3},
    {5, 2, 7}};

void DitheringOrdered(
    ImageRGB& output,
    ImageRGB const& input) {
    // Calculate the dimensions of the output image
    std::size_t outputWidth = input.GetSizeX() * 3;
    std::size_t outputHeight = input.GetSizeY() * 3;

    // Loop through the input image
    for (std::size_t x = 0; x < input.GetSizeX(); ++x) {
        for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
            glm::vec3 color = input.At(x, y);

            // Fill the 3x3 block with the same value
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    float val = (color.r + color.g + color.b) / 3 * 10 >= ditherMatrix[i][j] ? 1.0f : 0.0f;
                    output.At(x * 3 + i, y * 3 + j) = {val, val, val};
                }
            }
        }
    }
}

void DitheringErrorDiffuse(
    ImageRGB& output,
    ImageRGB const& input) {
    // Create a copy of the input image to store the error diffusion
    ImageRGB errorDiffusion(input);

    // Loop through the input image
    for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
        for (std::size_t x = 0; x < input.GetSizeX(); ++x) {
            glm::vec3 oldColor = errorDiffusion.At(x, y);

            // Calculate the threshold and error
            float threshold = 0.5f;
            glm::vec3 newColor = {
                oldColor.r >= threshold ? 1.0f : 0.0f,
                oldColor.g >= threshold ? 1.0f : 0.0f,
                oldColor.b >= threshold ? 1.0f : 0.0f};
            glm::vec3 error = oldColor - newColor;

            // Diffuse the error to neighboring pixels
            if (x + 1 < input.GetSizeX()) {
                errorDiffusion.At(x + 1, y) = glm::vec3(errorDiffusion.At(x + 1, y)) + error * (7.0f / 16.0f);
            }
            if (x > 0 && y + 1 < input.GetSizeY()) {
                errorDiffusion.At(x - 1, y + 1) = glm::vec3(errorDiffusion.At(x - 1, y + 1)) + error * (3.0f / 16.0f);
            }
            if (y + 1 < input.GetSizeY()) {
                errorDiffusion.At(x, y + 1) = glm::vec3(errorDiffusion.At(x, y + 1)) + error * (5.0f / 16.0f);
            }
            if (x + 1 < input.GetSizeX() && y + 1 < input.GetSizeY()) {
                errorDiffusion.At(x + 1, y + 1) = glm::vec3(errorDiffusion.At(x + 1, y + 1)) + error * (1.0f / 16.0f);
            }

            // Set the new color in the output image
            output.At(x, y) = newColor;
        }
    }
}
/******************* 2.Image Filtering *****************/
void Blur(ImageRGB& output, ImageRGB const& input) {
    int kernelSize = 3;
    float kernel[3][3] = {
        {1.0f / 9.0f, 1.0f / 9.0f, 1.0f / 9.0f},
        {1.0f / 9.0f, 1.0f / 9.0f, 1.0f / 9.0f},
        {1.0f / 9.0f, 1.0f / 9.0f, 1.0f / 9.0f}};

    for (std::size_t y = 1; y < input.GetSizeY() - 1; ++y) {
        for (std::size_t x = 1; x < input.GetSizeX() - 1; ++x) {
            glm::vec3 sum(0.0f);
            for (int ky = -1; ky <= 1; ++ky) {
                for (int kx = -1; kx <= 1; ++kx) {
                    sum += input.At(x + kx, y + ky) * kernel[ky + 1][kx + 1];
                }
            }
            output.At(x, y) = sum;
        }
    }
}

void Edge(ImageRGB& output, ImageRGB const& input) {
    int kernelSize = 3;
    float sobelX[3][3] = {
        {-1.0f, 0.0f, 1.0f},
        {-2.0f, 0.0f, 2.0f},
        {-1.0f, 0.0f, 1.0f}};

    float sobelY[3][3] = {
        {-1.0f, -2.0f, -1.0f},
        {0.0f, 0.0f, 0.0f},
        {1.0f, 2.0f, 1.0f}};

    for (std::size_t y = 1; y < input.GetSizeY() - 1; ++y) {
        for (std::size_t x = 1; x < input.GetSizeX() - 1; ++x) {
            glm::vec3 gradientX(0.0f);
            glm::vec3 gradientY(0.0f);

            for (int ky = -1; ky <= 1; ++ky) {
                for (int kx = -1; kx <= 1; ++kx) {
                    gradientX += input.At(x + kx, y + ky) * sobelX[ky + 1][kx + 1];
                    gradientY += input.At(x + kx, y + ky) * sobelY[ky + 1][kx + 1];
                }
            }

            glm::vec3 gradient = glm::sqrt(gradientX * gradientX + gradientY * gradientY);
            output.At(x, y) = gradient;
        }
    }
}

/******************* 3. Image Inpainting *****************/
void Inpainting(
    ImageRGB& output,
    ImageRGB const& inputBack,
    ImageRGB const& inputFront,
    const glm::ivec2& offset) {
    output = inputBack;
    std::size_t width = inputFront.GetSizeX();
    std::size_t height = inputFront.GetSizeY();
    glm::vec3* g = new glm::vec3[width * height];
    memset(g, 0, sizeof(glm::vec3) * width * height);
    // set boundary condition
    for (std::size_t y = 0; y < height; ++y) {
        // set boundary for (0, y), your code: g[y * width] = ?
        g[y * width] = inputBack.At(offset.x + 0, offset.y + y) - inputFront.At(0, y);
        // set boundary for (width - 1, y), your code: g[y * width + width - 1] = ?
        g[y * width + width - 1] = inputBack.At(offset.x + width - 1, offset.y + y) - inputFront.At(width - 1, y);
    }
    for (std::size_t x = 0; x < width; ++x) {
        // set boundary for (x, 0), your code: g[x] = ?
        g[x] = inputBack.At(offset.x + x, offset.y + 0) - inputFront.At(x, 0);
        // set boundary for (x, height - 1), your code: g[(height - 1) * width + x] = ?
        g[(height - 1) * width + x] = inputBack.At(offset.x + x, offset.y + height - 1) - inputFront.At(x, height - 1);
    }

    // Jacobi iteration, solve Ag = b
    for (int iter = 0; iter < 8000; ++iter) {
        for (std::size_t y = 1; y < height - 1; ++y)
            for (std::size_t x = 1; x < width - 1; ++x) {
                g[y * width + x] = (g[(y - 1) * width + x] + g[(y + 1) * width + x] + g[y * width + x - 1] + g[y * width + x + 1]);
                g[y * width + x] = g[y * width + x] * glm::vec3(0.25);
            }
    }

    for (std::size_t y = 0; y < inputFront.GetSizeY(); ++y)
        for (std::size_t x = 0; x < inputFront.GetSizeX(); ++x) {
            glm::vec3 color = g[y * width + x] + inputFront.At(x, y);
            output.At(x + offset.x, y + offset.y) = color;
        }
    delete[] g;
}

/******************* 4. Line Drawing *****************/

void DrawLine(
    ImageRGB& canvas,
    glm::vec3 const color,
    glm::ivec2 const p0,
    glm::ivec2 const p1) {
    int x0 = p0.x;
    int y0 = p0.y;
    int x1 = p1.x;
    int y1 = p1.y;

    bool steep = false;
    if (std::abs(x0 - x1) < std::abs(y0 - y1)) {
        std::swap(x0, y0);
        std::swap(x1, y1);
        steep = true;
    }

    if (x0 > x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    int dx = x1 - x0;
    int dy = std::abs(y1 - y0);
    int error = dx / 2;
    int ystep = (y0 < y1) ? 1 : -1;
    int y = y0;

    for (int x = x0; x <= x1; ++x) {
        if (steep) {
            canvas.At(y, x) = color;
        } else {
            canvas.At(x, y) = color;
        }
        error -= dy;
        if (error < 0) {
            y += ystep;
            error += dx;
        }
    }
}

/******************* 5. Triangle Drawing *****************/

// Helper function to draw a horizontal line segment
inline void DrawHorizontalLine(ImageRGB& canvas, glm::vec3 const color, int x1, int x2, int y) {
    if (x1 > x2) {
        std::swap(x1, x2);
    }
    for (int x = x1; x <= x2; ++x) {
        canvas.At(x, y) = color;
    }
}
void DrawTriangleFilled(
    ImageRGB& canvas,
    glm::vec3 const color,
    glm::ivec2 const p0,
    glm::ivec2 const p1,
    glm::ivec2 const p2) {
    // Sort the vertices in ascending order of y-coordinates
    glm::ivec2 vertices[3] = {p0, p1, p2};
    for (int i = 0; i < 2; ++i) {
        for (int j = i + 1; j < 3; ++j) {
            if (vertices[i].y > vertices[j].y) {
                std::swap(vertices[i], vertices[j]);
            }
        }
    }

    int x0 = vertices[0].x;
    int y0 = vertices[0].y;
    int x1 = vertices[1].x;
    int y1 = vertices[1].y;
    int x2 = vertices[2].x;
    int y2 = vertices[2].y;

    // Check for degenerate cases
    if (y0 == y2) {
        return;  // Flat top or flat bottom triangle, no need to draw
    }

    bool isFlatBottom = (y0 == y1);
    bool isFlatTop = (y1 == y2);

    float x01Step = 0;
    float x12Step = 0;
    float x02Step = 0;

    int x01 = x0;
    int x02 = x0;

    // Compute slopes for the two edges
    if (!isFlatBottom) {
        x01Step = float(x1 - x0) / std::abs(y1 - y0);
    }
    if (!isFlatTop) {
        x12Step = float(x2 - x1) / std::abs(y2 - y1);
    }
    x02Step = float(x2 - x0) / std::abs(y2 - y0);

    // Scan the bottom part of the triangle
    if (isFlatBottom) {
        x01 = x1;  // x01 is now x12, x01Step doesn't exist
        for (int y = y0; y < y2; ++y) {
            DrawHorizontalLine(canvas, color, x01, x02, y);
            x01 = x1 + int(x12Step * (y - y0));
            x02 = x0 + int(x02Step * (y - y0));
        }
        return;
    }
    // Scan the top part of the triangle
    else {
        for (int y = y0; y < y1; ++y) {
            DrawHorizontalLine(canvas, color, x01, x02, y);
            x01 = x0 + int(x01Step * (y - y0));
            x02 = x0 + int(x02Step * (y - y0));
        }
        if (isFlatTop) {
            return;
        }

        x01 = x1;  // x01 is now x12
        for (int y = y1; y < y2; ++y) {
            DrawHorizontalLine(canvas, color, x01, x02, y);
            x01 = x1 + int(x12Step * (y - y1));
            x02 = x0 + int(x02Step * (y - y0));
        }
    }
}

/******************* 6. Image Supersampling *****************/

void Supersample(
    ImageRGB& output,
    ImageRGB const& input,
    int rate) {
    int outputWidth = output.GetSizeX();
    int outputHeight = output.GetSizeY();
    int inputWidth = input.GetSizeX();
    int inputHeight = input.GetSizeY();

    // Loop through the output image
    for (int y = 0; y < outputHeight; ++y) {
        for (int x = 0; x < outputWidth; ++x) {
            glm::vec3 pixelColor(0.0f);

            // Loop through the supersampling grid within each pixel
            for (int sy = 0; sy < rate; ++sy) {
                for (int sx = 0; sx < rate; ++sx) {
                    float sampleX = static_cast<float>(x * rate + sx) / (outputWidth * rate);
                    float sampleY = static_cast<float>(y * rate + sy) / (outputHeight * rate);
                    int inputX = static_cast<int>(sampleX * inputWidth);
                    int inputY = static_cast<int>(sampleY * inputHeight);

                    // Sample the input image and accumulate colors
                    glm::vec3 sampleColor = input.At(inputX, inputY);
                    pixelColor += sampleColor;
                }
            }

            // Average the sampled colors
            pixelColor /= static_cast<float>(rate * rate);
            output.At(x, y) = pixelColor;
        }
    }
}

/******************* 7. Bezier Curve *****************/
glm::vec2 CalculateBezierPoint(
    std::span<glm::vec2> points,
    float const t) {
    int n = points.size() - 1;  // Number of control points

    // Base case: If there's only one point, return it
    if (n == 0) {
        return points[0];
    }

    // Create a temporary array to hold intermediate control points
    std::vector<glm::vec2> tempPoints(points.begin(), points.end());

    // De Casteljau's algorithm
    for (int r = 1; r <= n; ++r) {
        for (int i = 0; i <= n - r; ++i) {
            // Interpolate between two adjacent control points
            tempPoints[i] = (1 - t) * tempPoints[i] + t * tempPoints[i + 1];
        }
    }

    // The final point on the curve
    return tempPoints[0];
}
}  // namespace VCX::Labs::Drawing2D