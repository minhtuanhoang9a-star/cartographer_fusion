# Cấu trúc thư mục Cartographer

Các file và folder được chia theo tác dụng chính để dễ tra cứu.

| Tên | Giải thích |
|---|---|
| **1. Thông tin dự án và quản lý mã nguồn** | **Nhóm file mô tả dự án, giấy phép và quy tắc đóng góp.** |
| `README.rst` | Giới thiệu Cartographer, mục đích sử dụng và liên kết đến tài liệu. |
| `AUTHORS` | Danh sách tác giả và bên đóng góp cho dự án. |
| `LICENSE` | Giấy phép Apache 2.0 của Cartographer. |
| `CHANGELOG.rst` | Lịch sử thay đổi giữa các phiên bản. |
| `CONTRIBUTING.md` | Hướng dẫn đóng góp mã nguồn cho dự án. |
| `RELEASING.rst` | Quy trình chuẩn bị và phát hành phiên bản mới. |
| `.gitignore` | Danh sách file/thư mục Git không theo dõi, thường là file build hoặc file tạm. |
| **2. Build và dependency** | **Nhóm file dùng để biên dịch Cartographer và quản lý thư viện phụ thuộc.** |
| `CMakeLists.txt` | Cấu hình biên dịch bằng CMake; tìm dependency, build thư viện, test, protobuf và tài liệu. |
| `package.xml` | Khai báo package ROS, phiên bản, dependency và kiểu build CMake. |
| `BUILD.bazel` | Quy tắc build Cartographer bằng Bazel và khai báo các file cấu hình Lua. |
| `WORKSPACE` | Khai báo workspace và dependency bên ngoài cho Bazel. |
| `cartographer-config.cmake.in` | Mẫu file giúp package khác tìm và liên kết Cartographer qua CMake. |
| `bazel/` | Cấu hình dependency và quy tắc hỗ trợ build bằng Bazel. <br>- `repositories.bzl`: khai báo repository/dependency bên ngoài. <br>- `third_party/`: các file `*.BUILD` mô tả cách Bazel sử dụng thư viện ngoài như Ceres, Protobuf hoặc gRPC. |
| `cmake/` | Các tiện ích và module mở rộng cho CMake. <br>- `functions.cmake`: các hàm CMake dùng chung. <br>- `modules/`: module hỗ trợ tìm dependency hoặc cấu hình build. |
| `cartographer/BUILD.bazel` | Quy tắc build các thành phần C++ trong thư mục mã nguồn lõi. |
| **3. Docker và CI/CD** | **Nhóm file tạo môi trường build, kiểm thử và tích hợp liên tục.** |
| `Dockerfile.bionic` | Môi trường Docker dựa trên Ubuntu Bionic. |
| `Dockerfile.buster` | Môi trường Docker dựa trên Debian Buster. |
| `Dockerfile.focal` | Môi trường Docker dựa trên Ubuntu Focal. |
| `Dockerfile.stretch` | Môi trường Docker dựa trên Debian Stretch. |
| `Dockerfile.trusty.bazel` | Môi trường Docker Ubuntu Trusty dành cho build bằng Bazel. |
| `.dockerignore` | Danh sách file không đưa vào Docker build context. |
| `.bazelci/` | Cấu hình kiểm thử và build Cartographer trên Bazel CI. <br>- Có thể chứa thiết lập môi trường, phiên bản compiler và các job Bazel. |
| `.bazelrc` | Tùy chọn mặc định khi chạy các lệnh Bazel. |
| `azure-pipelines.yml` | Cấu hình pipeline CI/CD chạy trên Azure Pipelines. |
| `.travis.yml` | Cấu hình kiểm thử tự động trên Travis CI. |
| **4. Mã nguồn C++ lõi** | **Folder chứa thuật toán SLAM và các thư viện xử lý dữ liệu chính.** |
| `cartographer/` | Mã nguồn C++ lõi của hệ thống SLAM Cartographer. <br>- `mapping/`: local SLAM, submap, pose graph và tối ưu toàn cục. <br>- `sensor/`: dữ liệu laser, IMU, odometry, landmark và point cloud. <br>- `transform/`: pose, rigid transform và chuyển đổi giữa các frame. <br>- `io/`: đọc/ghi pbstream, bản đồ và point cloud. <br>- `common/`: tiện ích dùng chung như Lua, thread pool, time, math và sampler. <br>- `ground_truth/`: tạo ground truth và tính metric đánh giá trajectory. <br>- `metrics/`: counter, gauge và histogram theo dõi hoạt động hệ thống. <br>- `cloud/`: map builder server, gRPC và metrics cho chế độ cloud. |
| `cartographer/mapping/` | Module trung tâm thực hiện SLAM, quản lý trajectory, submap, scan matching và pose graph. <br>- `2d/`: thuật toán SLAM 2D. <br>- `3d/`: thuật toán SLAM 3D. <br>- `proto/`: dữ liệu protobuf để lưu/khôi phục trạng thái mapping. <br>- `*_test.cc`: kiểm thử mapping. |
| `cartographer/sensor/` | Định nghĩa và quản lý dữ liệu từ cảm biến. <br>- `range_data.*`: dữ liệu laser/range finder. <br>- `imu_data.*`: dữ liệu IMU. <br>- `odometry_data.*`: dữ liệu odometry. <br>- `point_cloud.*`: dữ liệu point cloud. <br>- `landmark_data.*`: dữ liệu landmark. <br>- `timed_point_cloud_data.*`: point cloud kèm thời gian. <br>- `proto/`: định dạng serialize dữ liệu cảm biến. |
| `cartographer/transform/` | Xử lý phép biến đổi tọa độ, pose và nội suy transform theo timestamp. <br>- `rigid_transform.*`: translation và rotation. <br>- `timestamped_transform.*`: transform gắn với thời gian. <br>- `transform_interpolation_buffer.*`: lưu và nội suy transform. <br>- `transform.*`: tiện ích xử lý transform. |
| `cartographer/io/` | Đọc, ghi, lọc và chuyển đổi dữ liệu bản đồ hoặc point cloud. <br>- `pbstream_main.cc`: công cụ xử lý file `.pbstream`. <br>- `proto_stream.*`: serialize/deserialize protobuf. <br>- `*_points_processor.*`: lọc và chuyển đổi point cloud. <br>- `pcd_writing_points_processor.*`: xuất PCD. <br>- `ply_writing_points_processor.*`: xuất PLY. <br>- `xyz_writing_points_processor.*`: xuất XYZ. <br>- `draw_trajectories.*`: vẽ trajectory. <br>- `submap_painter.*`: tạo ảnh từ submap. |
| `cartographer/common/` | Các tiện ích nền tảng được nhiều module sử dụng. <br>- `lua_parameter_dictionary.*`: đọc tham số Lua. <br>- `configuration_file_resolver.*`: tìm và nạp file cấu hình. <br>- `thread_pool.*`, `task.*`: thực thi công việc bất đồng bộ. <br>- `time.*`, `math.h`: xử lý thời gian và toán học. <br>- `fixed_ratio_sampler.*`: lấy mẫu theo tỷ lệ cố định. <br>- `proto/`: kiểu dữ liệu protobuf dùng chung. |
| `cartographer/ground_truth/` | Công cụ tạo và đánh giá ground truth cho trajectory. <br>- `autogenerate_ground_truth.*`: tạo ground truth tự động. <br>- `relations_text_file.*`: đọc/ghi quan hệ giữa các trajectory. <br>- `compute_relations_metrics_main.cc`: tính metric quan hệ trajectory. <br>- `proto/`: định dạng dữ liệu ground truth. |
| `cartographer/metrics/` | Hệ thống thu thập metrics nội bộ để theo dõi hiệu năng và trạng thái. <br>- `counter.*`: bộ đếm tăng dần. <br>- `gauge.*`: giá trị có thể thay đổi. <br>- `histogram.*`: thống kê phân bố giá trị. <br>- `register.*`: đăng ký và quản lý metrics. <br>- `family_factory.h`: tạo nhóm metric cùng loại. |
| `cartographer/cloud/` | Thành phần hỗ trợ chạy Cartographer theo mô hình client/server hoặc cloud. <br>- `map_builder_server_main.cc`: entry point của map builder server. <br>- `map_builder_server_interface.*`: interface giao tiếp với server. <br>- `map_builder_server_options.*`: tùy chọn khởi chạy server. <br>- `client/`: thành phần client. <br>- `metrics/`: metrics cho cloud. <br>- `proto/`: message protobuf giao tiếp. <br>- `internal/`: triển khai nội bộ. |
| **5. File cấu hình runtime** | **Nhóm file điều chỉnh tham số khi Cartographer chạy.** |
| `configuration_files/` | Các file cấu hình Lua mặc định của Cartographer. <br>- `map_builder.lua`: cấu hình map builder. <br>- `map_builder_server.lua`: cấu hình map builder server. <br>- `trajectory_builder.lua`: cấu hình chung cho trajectory builder. <br>- `trajectory_builder_2d.lua`: cấu hình trajectory builder 2D. <br>- `trajectory_builder_3d.lua`: cấu hình trajectory builder 3D. <br>- `pose_graph.lua`: cấu hình pose graph và global optimization. |
| **6. Tài liệu** | **Nhóm file xây dựng tài liệu kỹ thuật bằng Sphinx.** |
| `docs/` | Tài liệu của Cartographer. <br>- `source/`: nội dung tài liệu. <br>- `assets/`: hình ảnh và tài nguyên. <br>- `CMakeLists.txt`: cấu hình build tài liệu. |
| **7. Script cài đặt, build và kiểm thử** | **Nhóm script tự động hóa các công việc phát triển.** |
| `scripts/` | Script cài dependency, build, test và quản lý Docker/cache. <br>- `install_cartographer_cmake.sh`: build/cài bằng CMake. <br>- `install_cartographer_bazel.sh`: build/cài bằng Bazel. <br>- `install_debs_*.sh`: cài dependency dạng Debian package. <br>- `install_ceres.sh`, `install_abseil.sh`, `install_proto3.sh`: cài thư viện phụ thuộc. <br>- `build_test_asan.sh`: build và kiểm tra bằng AddressSanitizer. <br>- `update_configuration_doc.py`: cập nhật tài liệu cấu hình. <br>- `ctest_to_junit.py`: chuyển kết quả CTest sang JUnit. |
| **8. Quy ước loại file** | **Ý nghĩa chung của các phần mở rộng và tên file trong mã nguồn.** |
| `*.h` | Header C++, chứa khai báo class, hàm, struct và interface. |
| `*.cc` | File triển khai mã nguồn C++. |
| `*_test.cc` | Unit test. |
| `*_main.cc` | Entry point của chương trình thực thi độc lập. |
| `*.proto` | Định nghĩa message Protocol Buffers để serialize dữ liệu. |
| `internal/` | Thành phần triển khai nội bộ, không phải API công khai ổn định. |
