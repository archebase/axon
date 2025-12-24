# CMake generated Testfile for 
# Source directory: /Users/zhexuany/Repo/Axon/cpp/axon_uploader
# Build directory: /Users/zhexuany/Repo/Axon/cpp/build_uploader
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_upload_queue "/Users/zhexuany/Repo/Axon/cpp/build_uploader/test_upload_queue")
set_tests_properties(test_upload_queue PROPERTIES  _BACKTRACE_TRIPLES "/Users/zhexuany/Repo/Axon/cpp/axon_uploader/CMakeLists.txt;133;add_test;/Users/zhexuany/Repo/Axon/cpp/axon_uploader/CMakeLists.txt;137;add_uploader_test;/Users/zhexuany/Repo/Axon/cpp/axon_uploader/CMakeLists.txt;0;")
add_test(test_retry_handler "/Users/zhexuany/Repo/Axon/cpp/build_uploader/test_retry_handler")
set_tests_properties(test_retry_handler PROPERTIES  _BACKTRACE_TRIPLES "/Users/zhexuany/Repo/Axon/cpp/axon_uploader/CMakeLists.txt;133;add_test;/Users/zhexuany/Repo/Axon/cpp/axon_uploader/CMakeLists.txt;140;add_uploader_test;/Users/zhexuany/Repo/Axon/cpp/axon_uploader/CMakeLists.txt;0;")
add_test(test_state_manager "/Users/zhexuany/Repo/Axon/cpp/build_uploader/test_state_manager")
set_tests_properties(test_state_manager PROPERTIES  _BACKTRACE_TRIPLES "/Users/zhexuany/Repo/Axon/cpp/axon_uploader/CMakeLists.txt;133;add_test;/Users/zhexuany/Repo/Axon/cpp/axon_uploader/CMakeLists.txt;143;add_uploader_test;/Users/zhexuany/Repo/Axon/cpp/axon_uploader/CMakeLists.txt;0;")
subdirs("axon_mcap")
