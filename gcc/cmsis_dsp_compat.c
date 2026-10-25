#include "arm_math.h"
#include <math.h>
#include <string.h>

void arm_mat_init_f32(arm_matrix_instance_f32 *matrix, uint16_t rows,
                      uint16_t columns, float32_t *data)
{
    matrix->numRows = rows;
    matrix->numCols = columns;
    matrix->pData = data;
}

static arm_status same_size(const arm_matrix_instance_f32 *a,
                            const arm_matrix_instance_f32 *b,
                            const arm_matrix_instance_f32 *out)
{
    return a->numRows == b->numRows && a->numCols == b->numCols &&
           a->numRows == out->numRows && a->numCols == out->numCols
               ? ARM_MATH_SUCCESS : ARM_MATH_SIZE_MISMATCH;
}

arm_status arm_mat_add_f32(const arm_matrix_instance_f32 *a,
                           const arm_matrix_instance_f32 *b,
                           arm_matrix_instance_f32 *out)
{
    if (same_size(a, b, out) != ARM_MATH_SUCCESS)
        return ARM_MATH_SIZE_MISMATCH;
    uint32_t count = (uint32_t)a->numRows * a->numCols;
    for (uint32_t i = 0; i < count; ++i)
        out->pData[i] = a->pData[i] + b->pData[i];
    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_sub_f32(const arm_matrix_instance_f32 *a,
                           const arm_matrix_instance_f32 *b,
                           arm_matrix_instance_f32 *out)
{
    if (same_size(a, b, out) != ARM_MATH_SUCCESS)
        return ARM_MATH_SIZE_MISMATCH;
    uint32_t count = (uint32_t)a->numRows * a->numCols;
    for (uint32_t i = 0; i < count; ++i)
        out->pData[i] = a->pData[i] - b->pData[i];
    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_mult_f32(const arm_matrix_instance_f32 *a,
                            const arm_matrix_instance_f32 *b,
                            arm_matrix_instance_f32 *out)
{
    if (a->numCols != b->numRows || out->numRows != a->numRows ||
        out->numCols != b->numCols)
        return ARM_MATH_SIZE_MISMATCH;

    for (uint16_t row = 0; row < out->numRows; ++row) {
        for (uint16_t column = 0; column < out->numCols; ++column) {
            float32_t sum = 0.0f;
            for (uint16_t i = 0; i < a->numCols; ++i)
                sum += a->pData[row * a->numCols + i] *
                       b->pData[i * b->numCols + column];
            out->pData[row * out->numCols + column] = sum;
        }
    }
    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_trans_f32(const arm_matrix_instance_f32 *input,
                             arm_matrix_instance_f32 *out)
{
    if (out->numRows != input->numCols || out->numCols != input->numRows)
        return ARM_MATH_SIZE_MISMATCH;
    for (uint16_t row = 0; row < input->numRows; ++row)
        for (uint16_t column = 0; column < input->numCols; ++column)
            out->pData[column * out->numCols + row] =
                input->pData[row * input->numCols + column];
    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_inverse_f32(const arm_matrix_instance_f32 *input,
                               arm_matrix_instance_f32 *out)
{
    if (input->numRows != input->numCols || out->numRows != input->numRows ||
        out->numCols != input->numCols || input->numRows > 6U)
        return ARM_MATH_SIZE_MISMATCH;

    const uint16_t n = input->numRows;
    float32_t work[6][12] = {{0}};
    for (uint16_t row = 0; row < n; ++row) {
        for (uint16_t column = 0; column < n; ++column)
            work[row][column] = input->pData[row * n + column];
        work[row][n + row] = 1.0f;
    }

    for (uint16_t pivot = 0; pivot < n; ++pivot) {
        uint16_t best = pivot;
        for (uint16_t row = pivot + 1; row < n; ++row)
            if (fabsf(work[row][pivot]) > fabsf(work[best][pivot]))
                best = row;
        if (fabsf(work[best][pivot]) < 1.0e-12f)
            return ARM_MATH_SINGULAR;
        if (best != pivot) {
            for (uint16_t column = 0; column < 2U * n; ++column) {
                float32_t value = work[pivot][column];
                work[pivot][column] = work[best][column];
                work[best][column] = value;
            }
        }
        float32_t divisor = work[pivot][pivot];
        for (uint16_t column = 0; column < 2U * n; ++column)
            work[pivot][column] /= divisor;
        for (uint16_t row = 0; row < n; ++row) {
            if (row == pivot)
                continue;
            float32_t factor = work[row][pivot];
            for (uint16_t column = 0; column < 2U * n; ++column)
                work[row][column] -= factor * work[pivot][column];
        }
    }

    for (uint16_t row = 0; row < n; ++row)
        memcpy(&out->pData[row * n], &work[row][n], n * sizeof(float32_t));
    return ARM_MATH_SUCCESS;
}

float32_t arm_sin_f32(float32_t value)
{
    return sinf(value);
}

float32_t arm_cos_f32(float32_t value)
{
    return cosf(value);
}
