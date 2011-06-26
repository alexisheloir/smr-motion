PROJECT(SAMSARA_NEWMAT)

INCLUDE_DIRECTORIES()

ADD_LIBRARY(newmat STATIC 
			bandmat.cpp
			myexcept.cpp
			newmat1.cpp
			newmat2.cpp
			newmat3.cpp
			newmat4.cpp 
			newmat5.cpp
			newmat6.cpp
			newmat7.cpp
			newmat8.cpp
			newmat9.cpp
			newmatex.cpp
			newmatrm.cpp
			submat.cpp
			cholesky.cpp
			evalue.cpp
			fft.cpp
			hholder.cpp
			jacobi.cpp
			sort.cpp
			svd.cpp
			newfft.cpp
			nm_misc.cpp
			newmatnl.cpp
			solution.cpp

			newmat.h
			newmatio.h	
			newmatap.h
			include.h
)

SOURCE_GROUP(Headers REGULAR_EXPRESSION "[.]h$")

			

