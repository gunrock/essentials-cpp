/**
 * @file load.hxx
 * @author Muhammad Osama (mosama@ucdavis.edu)
 * @brief Matrix Market file loader.
 * @version 0.1
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

#include <string>
#include <assert.h>
#include <externals/mmio.hxx>

namespace essentials {

/**
 * @brief Matrix Market format supports two kind of formats, a sparse coordinate
 * format and a dense array format.
 *
 */
enum matrix_market_format_t { coordinate, array };

/**
 * @brief Data type defines the type of data presented in the file, things like,
 * are they real numbers, complex (real and imaginary), pattern (do not have
 * weights/nonzero-values), etc.
 *
 */
enum matrix_market_data_t { real, complex, pattern, integer };

/**
 * @brief Storage scheme defines the storage structure, symmetric matrix for
 * example will be symmetric over the diagonal. Skew is skew symmetric. Etc.
 *
 */
enum matrix_market_storage_scheme_t { general, hermitian, symmetric, skew };

/**
 * @brief Reads a MARKET graph from an input-stream
 * into a specified sparse format
 *
 * Here is an example of the matrix market format
 * +----------------------------------------------+
 * |%%MatrixMarket matrix coordinate real general | <--- header line
 * |%                                             | <--+
 * |% comments                                    |    |-- 0 or more comments
 * |%                                             | <--+
 * |  M N L                                       | <--- rows, columns, entries
 * |  I1 J1 A(I1, J1)                             | <--+
 * |  I2 J2 A(I2, J2)                             |    |
 * |  I3 J3 A(I3, J3)                             |    |-- L lines
 * |     . . .                                    |    |
 * |  IL JL A(IL, JL)                             | <--+
 * +----------------------------------------------+
 *
 * Indices are 1-based i.2. A(1,1) is the first element.
 */
template <typename vertex_t, typename edge_t, typename weight_t>
struct matrix_market_t {
  // typedef FILE* file_t;
  // typedef MM_typecode matrix_market_code_t;

  using file_t = FILE*;
  using matrix_market_code_t = MM_typecode;

  std::string filename;
  std::string dataset;

  int num_rows, num_columns, num_nonzeros;

  // COO vectors
  std::vector<vertex_t> row_indices;
  std::vector<vertex_t> column_indices;
  std::vector<weight_t> values;

  // CSR vectors
  std::vector<edge_t> Ap;
  std::vector<vertex_t> Aj;
  std::vector<weight_t> Ax;

  // Dataset characteristics
  matrix_market_code_t code;              // (ALL INFORMATION)
  matrix_market_format_t format;          // Sparse coordinate or dense array
  matrix_market_data_t data;              // Data type
  matrix_market_storage_scheme_t scheme;  // Storage scheme

  /**
   * @brief Loads the given .mtx file into a coordinate (COO) and
   * compressed-sparse row (CSR) formats.
   *
   * @param _filename input file name (.mtx)
   */
  matrix_market_t(std::string _filename) : filename(_filename) {
    file_t file;

    // Load MTX information
    if ((file = fopen(filename.c_str(), "r")) == NULL) {
      std::cerr << "File could not be opened: " << filename << std::endl;
      exit(1);
    }

    if (mm_read_banner(file, &code) != 0) {
      std::cerr << "Could not process Matrix Market banner" << std::endl;
      exit(1);
    }

    if ((mm_read_mtx_crd_size(file, &num_rows, &num_columns, &num_nonzeros)) !=
        0) {
      std::cerr << "Could not read file info (M, N, NNZ)" << std::endl;
      exit(1);
    }

    row_indices.resize(num_nonzeros);
    column_indices.resize(num_nonzeros);
    values.resize(num_nonzeros);

    if (mm_is_coordinate(code))
      format = matrix_market_format_t::coordinate;
    else
      format = matrix_market_format_t::array;

    if (mm_is_pattern(code)) {
      data = matrix_market_data_t::pattern;

      // pattern matrix defines sparsity pattern, but not values
      for (vertex_t i = 0; i < num_nonzeros; ++i) {
        assert(fscanf(file, " %d %d \n", &(row_indices[i]),
                      &(column_indices[i])) == 2);
        row_indices[i]--;  // adjust from 1-based to 0-based indexing
        column_indices[i]--;
        values[i] = (weight_t)1.0;  // use value 1.0 for all nonzero entries
      }
    } else if (mm_is_real(code) || mm_is_integer(code)) {
      if (mm_is_real(code))
        data = matrix_market_data_t::real;
      else
        data = matrix_market_data_t::integer;

      for (vertex_t i = 0; i < num_nonzeros; ++i) {
        vertex_t I = 0;
        vertex_t J = 0;
        double V = 0.0f;

        assert(fscanf(file, " %d %d %lf \n", &I, &J, &V) == 3);

        row_indices[i] = (vertex_t)I - 1;
        column_indices[i] = (vertex_t)J - 1;
        values[i] = (weight_t)V;
      }
    } else {
      std::cerr << "Unrecognized matrix market format type" << std::endl;
      exit(1);
    }

    if (mm_is_symmetric(code)) {  // duplicate off diagonal entries
      scheme = matrix_market_storage_scheme_t::symmetric;
      vertex_t off_diagonals = 0;
      for (vertex_t i = 0; i < num_nonzeros; ++i) {
        if (row_indices[i] != column_indices[i])
          ++off_diagonals;
      }

      vertex_t _nonzeros = 2 * off_diagonals + (num_nonzeros - off_diagonals);

      std::vector<vertex_t> new_I(_nonzeros);
      std::vector<vertex_t> new_J(_nonzeros);
      std::vector<weight_t> new_V(_nonzeros);

      vertex_t* _I = new_I.data();
      vertex_t* _J = new_J.data();
      weight_t* _V = new_V.data();

      vertex_t ptr = 0;
      for (vertex_t i = 0; i < num_nonzeros; ++i) {
        if (row_indices[i] != column_indices[i]) {
          _I[ptr] = row_indices[i];
          _J[ptr] = column_indices[i];
          _V[ptr] = values[i];
          ++ptr;
          _J[ptr] = row_indices[i];
          _I[ptr] = column_indices[i];
          _V[ptr] = values[i];
          ++ptr;
        } else {
          _I[ptr] = row_indices[i];
          _J[ptr] = column_indices[i];
          _V[ptr] = values[i];
          ++ptr;
        }
      }
      row_indices = new_I;
      column_indices = new_J;
      values = new_V;
      num_nonzeros = _nonzeros;
    }  // end symmetric case

    fclose(file);

    // Convert from COO to CSR.
    Ap.resize(num_rows + 1);
    Aj.resize(num_nonzeros);
    Ax.resize(num_nonzeros);

    // compute number of non-zero entries per row of A.
    for (edge_t n = 0; n < num_nonzeros; ++n) {
      ++Ap[row_indices[n]];
    }

    // cumulative sum the nnz per row to get row_offsets[].
    for (vertex_t i = 0, sum = 0; i < num_rows; ++i) {
      vertex_t temp = Ap[i];
      Ap[i] = sum;
      sum += temp;
    }
    Ap[num_rows] = num_nonzeros;

    // write coordinate column indices and nonzero values into CSR's
    // column indices and nonzero values.
    for (edge_t n = 0; n < num_nonzeros; ++n) {
      vertex_t row = row_indices[n];
      vertex_t dest = Ap[row];

      Aj[dest] = column_indices[n];
      Ax[dest] = values[n];

      ++Ap[row];
    }

    for (vertex_t i = 0, last = 0; i <= num_rows; ++i) {
      vertex_t temp = Ap[i];
      Ap[i] = last;
      last = temp;
    }
  }
};

}  // namespace essentials
