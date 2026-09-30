basjoo_cxx_library(
  NAME
    math_index_pair
  HDRS
    "index_pair.hpp"
  DEPS
    zpp_bits
)

basjoo_cxx_library(
  NAME
    math_sparse_traits
  HDRS
    "sparse_traits.hpp"
  DEPS
    common_aligned_allocator
    math_concepts
    math_index_pair
)

basjoo_cxx_library(
  NAME
    math_sparse_matrix_proxy
  HDRS
    "sparse_matrix_proxy.hpp"
  DEPS
    msft_proxy4
    zpp_bits
    math_concepts
    math_dok_matrix
    math_lil_matrix
    math_coo_matrix
    math_csc_matrix
    math_csr_matrix
)

basjoo_cxx_library(
  NAME
    math_dok_matrix
  HDRS
    "dok_matrix.hpp"
  DEPS
    zpp_bits
    math_concepts
    math_sparse_traits
)

basjoo_cxx_library(
  NAME
    math_lil_matrix
  HDRS
    "lil_matrix.hpp"
  DEPS
    zpp_bits
    common_aligned_allocator
    math_concepts
    math_sparse_traits
)

basjoo_cxx_library(
  NAME
    math_coo_matrix
  HDRS
    "coo_matrix.hpp"
  DEPS
    zpp_bits
    math_concepts
    math_sparse_traits
)

basjoo_cxx_library(
  NAME
    math_csc_matrix
  HDRS
    "csc_matrix.hpp"
  DEPS
    zpp_bits
    math_concepts
    math_index_pair
    math_sparse_traits
)

basjoo_cxx_library(
  NAME
    math_csr_matrix
  HDRS
    "csr_matrix.hpp"
  DEPS
    zpp_bits
    math_concepts
    math_index_pair
    math_sparse_traits
)

basjoo_cxx_library(
  NAME
    math_qdldl_dcmp
  HDRS
    "qdldl_dcmp.hpp"
  DEPS
    "$<IF:$<BOOL:${BUILD_SHARED_LIBS}>,qdldl,qdldlstatic>"
    math_concepts
    math_csc_matrix
)

if(BASJOO_USE_BOOST_UNORDERED)
  set_property(TARGET math_sparse_traits math_index_pair math_dok_matrix math_lil_matrix math_coo_matrix math_csc_matrix math_csr_matrix
    APPEND PROPERTY
      INTERFACE_COMPILE_DEFINITIONS "BASJOO_USE_BOOST_UNORDERED=1"
  )
  set_property(TARGET math_sparse_traits math_index_pair
    APPEND PROPERTY
      INTERFACE_LINK_LIBRARIES "Boost::unordered"
  )
endif()
