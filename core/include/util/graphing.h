#pragma once
#include <vector>
#include <type_traits>

namespace utility {

    void plot2d(const std::vector<std::vector<float>>& data);

    class grapher {
    private:
        std::vector<std::vector<float>> data;

    public:
        grapher() = default;

        template<typename... Args>
        void dataEntry(Args... args) {
            std::vector<float> row;
            row.reserve(sizeof...(args));
            (row.push_back(static_cast<float>(args)), ...);
            data.push_back(std::move(row));
        }

        void plot() {
            plot2d(data);
        }

        std::vector<std::vector<float>> copyData() const {
            return data;
        }

        void clear() {
            data.clear();
        }
    };
}
