cmake_minimum_required(VERSION 2.8.3)
project(cl_yaml)
find_package(catkin REQUIRED
cl_libyaml
alexandria
cl_ppcre
parse_number
)
catkin_package()
foreach(dir
src
tests)
install(DIRECTORY ${dir}
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
PATTERN ".svn" EXCLUDE
PATTERN ".git" EXCLUDE)
endforeach()
# install (FILES cram-utilities.asd cram-utilities-tests.asd
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
