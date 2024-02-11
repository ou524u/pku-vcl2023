#include "Labs/5-Visualization/tasks.h"

#include <assert.h>
#include <numbers>

#include <iostream>
#include "Labs/5-Visualization/tasks.h"

using VCX::Labs::Common::ImageRGB;

namespace VCX::Labs::Visualization {

float data_i(Car c, int i) {
    switch (i) {
        case 0:
            return c.cylinders;
        case 1:
            return c.displacement;
        case 2:
            return c.weight;
        case 3:
            return c.horsepower;
        case 4:
            return c.acceleration;
        case 5:
            return c.mileage;
        case 6:
            return c.year;
        default:
            return 0;
    }
}

// State class to handle visualization and interaction updates
class CoordinateStates {
   public:
    CoordinateStates(std::vector<Car> const& data)
        : data(data) {}

    bool Update(InteractProxy const& proxy) {
        // Update state based on user interaction (for example, check if dragging)
        // For simplicity, this example always returns true to force repainting
        return true;
    }

    void Paint(ImageRGB& input) {
        // Clear the background
        SetBackGround(input, glm::vec4(1.0f));

        // Draw parallel coordinates lines
        DrawOthers(input);
        DrawParallelCoordinates(input);

        // Example of printing text
        // PrintText(input, glm::vec4(0.0, 0.0, 0.0, 1.0), glm::vec2(0.1, 0.9), 20.0, "Parallel Coordinates");

        // You can add more drawing logic based on your specific requirements
    }

   private:
    std::vector<Car> const& data;
    float datarange[2][7] = {
        {2, 29, 1260, 27, 6, 5, 68},
        {9, 494, 5493, 249, 27, 51, 84}};

    void DrawParallelCoordinates(ImageRGB& input) {
        // Example: Drawing parallel lines for each car's data
        int numCars = static_cast<int>(data.size());

        for (int i = 0; i < numCars; ++i) {
            float ratio = static_cast<float>(i) / numCars;  // X position based on data index
            for (int j = 0; j < 6; j++) {
                float y1, y2;
                y1 = 0.9 - 0.8 * (data_i(data[i], j) - datarange[0][j]) / (datarange[1][j] - datarange[0][j]);
                y2 = 0.9 - 0.8 * (data_i(data[i], j + 1) - datarange[0][j + 1]) / (datarange[1][j + 1] - datarange[0][j + 1]);

                // Draw lines connecting each data point for a car
                DrawLine(input, glm::vec4(0.8 * ratio, 0.1, 0.8 - 1.0 * ratio, 0.2), glm::vec2(0.08 + j * 0.14, y1), glm::vec2(0.22 + j * 0.14, y2), 2.0);

                // Draw points representing each data point on the line
                // DrawFilledCircle(input, glm::vec4(0.0, 0.0, 0.0, 0.6), glm::vec2(0.08 + j * 0.14, y1), 0.01);
                // DrawFilledCircle(input, glm::vec4(0.0, 0.0, 0.0, 0.6), glm::vec2(0.22 + j * 0.14, y2), 0.01);
            }
        }
    }
    void DrawOthers(ImageRGB& input) {
        const char* datarange_str[2][7] = {
            {"2", "29", "1260", "27", "6", "5", "68"},
            {"9", "494", "5493", "249", "27", "51", "84"}};
        const char* text[7] = {"cylinders", "displacement", "weight", "horsepower", "acceleration(0-60mph)", "mileage", "year"};
        for (int i = 0; i < 7; i++) {
            DrawFilledRect(input, glm::vec4(0.2, 0.2, 0.2, 0.3), glm::vec2(0.06 + i * 0.14, 0.1), glm::vec2(0.04, 0.8));
            DrawFilledRect(input, glm::vec4(0.5, 0.5, 0.5, 0.5), glm::vec2(0.075 + i * 0.14, 0.1), glm::vec2(0.01, 0.8));
            PrintText(input, glm::vec4(0, 0, 0, 1), glm::vec2(0.08 + i * 0.14, 0.06), 0.015, text[i]);
            PrintText(input, glm::vec4(0, 0, 0, 1), glm::vec2(0.08 + i * 0.14, 0.92), 0.015, datarange_str[0][i]);
            PrintText(input, glm::vec4(0, 0, 0, 1), glm::vec2(0.08 + i * 0.14, 0.08), 0.015, datarange_str[1][i]);
        }
    }

    float MapToRange(float value, float minValue, float maxValue) {
        // Map a value from a given range to the range [0, 1]
        return (value - minValue) / (maxValue - minValue);
    }
};

bool PaintParallelCoordinates(ImageRGB& input, InteractProxy const& proxy, std::vector<Car> const& data, bool force) {
    static CoordinateStates states(data);

    // Update the state based on user interaction
    bool change = states.Update(proxy);

    // Skip repainting if force is false and there's no change
    if (!force && !change) {
        return false;
    }

    // Visualize based on the updated state
    states.Paint(input);

    return true;
}

void LIC(ImageRGB& output, Common::ImageRGB const& noise, VectorField2D const& field, int const& step) {
    // your code here
    for (int x = 0; x < output.GetSizeX(); x++) {
        for (int y = 0; y < output.GetSizeY(); y++) {
            // staring point (x,y)
            glm::vec3 o_Pixel(0.0f);
            float o_weight = 0;

            float curr_x = x;
            float curr_y = y;
            float scalar = 1;

            for (int i = 0; i < step; i++) {
                glm::vec2 dxy = field.At(std::round(curr_x), std::round(curr_y));
                if (dxy[0] == 0 && dxy[1] == 0) {
                    break;
                }
                if (dxy[0] > 0) {
                    scalar = (std::floor(curr_x) + 1.0f - curr_x) / dxy[0];
                } else {
                    scalar = (curr_x - (std::ceil(curr_x) - 1)) / (-dxy[0]);
                }
                if (dxy[1] > 0) {
                    scalar = std::min(scalar, (std::floor(curr_y) + 1.0f - curr_y) / dxy[1]);
                } else {
                    scalar = std::min(scalar, (curr_y - (std ::ceil(curr_y) - 1)) / (-dxy[1]));
                }
                curr_x += scalar * dxy[0];
                curr_y += scalar * dxy[1];
                if (std::round(curr_x) >= 0 && std::round(curr_x) < noise.GetSizeX() && std::round(curr_y) >= 0 && std::round(curr_y) < noise.GetSizeY()) {
                    // float weight = std::pow(std::cos(0.46 * i), 2);
                    float weight = 1;
                    o_Pixel += noise.At(std::round(curr_x), std::round(curr_y)) * weight;
                    o_weight += weight;
                } else {
                    break;
                }
            }
            curr_x = x;
            curr_y = y;

            for (int i = 0; i < step; i++) {
                glm::vec2 dxy = field.At(std::round(curr_x), std::round(curr_y));
                dxy *= -1;
                if (dxy[0] == 0 && dxy[1] == 0) {
                    break;
                }
                if (dxy[0] > 0) {
                    scalar = (std::floor(curr_x) + 1.0f - curr_x) / dxy[0];
                } else {
                    scalar = (curr_x - (std::ceil(curr_x) - 1)) / (-dxy[0]);
                }
                if (dxy[1] > 0) {
                    scalar = std::min(scalar, (std::floor(curr_y) + 1.0f - curr_y) / dxy[1]);
                } else {
                    scalar = std::min(scalar, (curr_y - (std ::ceil(curr_y) - 1)) / (-dxy[1]));
                }
                curr_x += scalar * dxy[0];
                curr_y += scalar * dxy[1];
                if (std::round(curr_x) >= 0 && std::round(curr_x) < noise.GetSizeX() && std::round(curr_y) >= 0 && std::round(curr_y) < noise.GetSizeY()) {
                    // float weight = std::pow(std::cos(0.46 * i), 2);
                    float weight = 1;
                    o_Pixel += noise.At(std::round(curr_x), std::round(curr_y)) * weight;
                    o_weight += weight;
                } else {
                    break;
                }
            }

            output.At(x, y) = o_Pixel / o_weight;
        }
    }
}
};  // namespace VCX::Labs::Visualization