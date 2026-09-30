target("math_index_pair")
    set_kind("headeronly")
    add_includedirs("../../..")
    add_headerfiles("index_pair.hpp")
    add_packages("zpp_bits", {public = true})
    if is_config("basjoo_use_boost_unordered", true) then
        add_defines("BASJOO_USE_BOOST_UNORDERED=1", {public = true})
        add_packages("boost", {public = true})
    end
target_end()

target("math_sparse_traits")
    set_kind("headeronly")
    add_includedirs("../../..")
    add_headerfiles("sparse_traits.hpp")
    add_deps("common_aligned_allocator", "math_concepts", "math_index_pair")
    if is_config("basjoo_use_boost_unordered", true) then
        add_defines("BASJOO_USE_BOOST_UNORDERED=1", {public = true})
        add_packages("boost", {public = true})
    end
target_end()

target("math_sparse_matrix_proxy")
    set_kind("headeronly")
    add_includedirs("../../..")
    add_headerfiles("sparse_matrix_proxy.hpp")
    add_packages("msft_proxy4", "zpp_bits", {public = true})
    add_deps("math_concepts", "math_dok_matrix", "math_lil_matrix", "math_coo_matrix", "math_csc_matrix", "math_csr_matrix")
target_end()

target("math_dok_matrix")
    set_kind("headeronly")
    add_includedirs("../../..")
    add_headerfiles("dok_matrix.hpp")
    add_packages("zpp_bits", {public = true})
    add_deps("math_concepts", "math_sparse_traits")
    if is_config("basjoo_use_boost_unordered", true) then
        add_defines("BASJOO_USE_BOOST_UNORDERED=1", {public = true})
    end
target_end()

target("math_lil_matrix")
    set_kind("headeronly")
    add_includedirs("../../..")
    add_headerfiles("lil_matrix.hpp")
    add_packages("zpp_bits", {public = true})
    add_deps("common_aligned_allocator", "math_concepts", "math_sparse_traits")
    if is_config("basjoo_use_boost_unordered", true) then
        add_defines("BASJOO_USE_BOOST_UNORDERED=1", {public = true})
    end
target_end()

target("math_coo_matrix")
    set_kind("headeronly")
    add_includedirs("../../..")
    add_headerfiles("coo_matrix.hpp")
    add_packages("zpp_bits", {public = true})
    add_deps("math_concepts", "math_sparse_traits")
    if is_config("basjoo_use_boost_unordered", true) then
        add_defines("BASJOO_USE_BOOST_UNORDERED=1", {public = true})
    end
target_end()

target("math_csc_matrix")
    set_kind("headeronly")
    add_includedirs("../../..")
    add_headerfiles("csc_matrix.hpp")
    add_packages("zpp_bits", {public = true})
    add_deps("math_concepts", "math_index_pair", "math_sparse_traits")
    if is_config("basjoo_use_boost_unordered", true) then
        add_defines("BASJOO_USE_BOOST_UNORDERED=1", {public = true})
    end
target_end()

target("math_csr_matrix")
    set_kind("headeronly")
    add_includedirs("../../..")
    add_headerfiles("csr_matrix.hpp")
    add_packages("zpp_bits", {public = true})
    add_deps("math_concepts", "math_index_pair", "math_sparse_traits")
    if is_config("basjoo_use_boost_unordered", true) then
        add_defines("BASJOO_USE_BOOST_UNORDERED=1", {public = true})
    end
target_end()

target("math_qdldl_dcmp")
    set_kind("headeronly")
    add_includedirs("../../..")
    add_headerfiles("qdldl_dcmp.hpp")
    add_packages("qdldl", {public = true})
    add_deps("math_concepts", "math_csc_matrix")
target_end()
