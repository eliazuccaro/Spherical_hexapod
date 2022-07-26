"""
      iDynTree.py - Lie Geometric Algorithms for the Kinematics, Dynamics and
                      Control of Multibody Systems (MBS)            
"""
# VERSION 3.1

import numpy as np
import scipy.linalg
from numpy.linalg import svd
from numpy import linalg as LA


# use in this way: vectorize([0, 0]) or vectorize([0])
def vectorize(value_list):
    return np.transpose((np.array([value_list])))

"""Matrix Utility Functions--------------------------------------------------"""

"""appendColumns(mat1, mat2) gives a new matrix composed of the
submatrices mat1, mat2, by joining their columns. The submatrices must
all have the same number of columns."""
def appendColumns (mat1, mat2):
    a=np.asarray(mat1)
    b=np.asarray(mat2)
    if a.shape[1]==b.shape[1]:
        return np.concatenate((a, b), axis=0)
    else:
        raise Exception("FUNCTION appendColumns: the columns of the matrices are not equal")
    
"""appendRows(mat1, mat2) gives a new matrix composed of the submatrices
mat1, mat2, by joining their rows. The submatrices must all have the same
number of rows."""
def appendRows (mat1, mat2):
    a=np.asarray(mat1)
    b=np.asarray(mat2)
    if a.shape[0]==b.shape[0]:
        return np.concatenate((a, b), axis=1)
    else:
        raise Exception("FUNCTION appendRows: the rows of the matrices are not equal")

"""stakCols(mat1, mat2) gives a new matrix obtained by stacking the columns 
of the submatrices mat1, mat2. The submatrices must all have the same number
of rows."""
def stackCols (mat1, mat2):
    return appendRows (mat1, mat2)

"""stackRows(mat1, mat2) gives a new matrix obtained by stacking the rows
of the submatrices mat1, mat2. The submatrices must all have the same number 
of columns."""
def stackRows (mat1, mat2):
    return appendColumns(mat1, mat2)

"""blockDiag(mat1, mat2) gives a block-diagonal matrix composed of the matrices mat1, mat2."""
def blockDiag (mat1, mat2):
    a=np.asarray(mat1)
    b=np.asarray(mat2)
    return scipy.linalg.block_diag(a,b)


"""takeRows(mat, rowStart, rowFinish) takes the rows of matrix mat from index rowStart to index rowFinish"""
def takeRows (mat, rowStart, rowFinish):
    a=np.asarray(mat)
    return a[rowStart:(rowFinish+1),:]

"""TakeColumns(mat,  columnsStart, columnsFinish) takes the columns of matrix mat 
from index columnsStart to index columnsFinish"""
def takeColumns (mat, columnStart, columnFinish):
    a=np.asarray(mat)
    return a[:,columnStart:(columnFinish+1)]

"""takeMatrix(mat, rowStart, columnStart, rowFinish, columnFinish) gives the submatrix 
starting at position (rowStart,columnStart) and ending at position (rowFinish,columnFinish)"""
def takeMatrix (mat, rowStart, columnStart, rowFinish, columnFinish):
    a=np.asarray(mat)
    return a[rowStart:(rowFinish+1),columnStart:(columnFinish+1)]

"""subMatrix(mat, rowStart, columnStart, m, n) gives the submatrix of dimension {m, n} starting
at position (rowStart,columnStart) in mat."""
def subMatrix (mat, rowStart, columnStart, m, n):
    a=np.asarray(mat)
    return a[rowStart:(rowStart+m),columnStart:(columnStart+n)]

"""zeroMatrix(n) gives the nxn zero matrix."""
def zeroMatrix (n):
    dim=(n,n)
    return np.zeros(dim)
    
"""eye(n) gives the nxn identity matrix."""
def eye (n):
    return np.eye(n)

"""selectionMatrix(n, indexRows)
returns the identity matrix n*n with only rows indicated in indexRows array"""
def selectionMatrix (m, indexRows):
    indexRows=np.int_(indexRows) 
    temp=eye(m)
    return temp[indexRows,:]


"""selectionMatrixColumns(n, indexColumns)
returns the identity matrix n*n with only columns indicated in indexColumns array"""
def selectionMatrixColumns (n, indexColumns):
    indexColumns=np.int_(indexColumns)
    temp=eye(n)
    return temp[:,indexColumns]
        
"""magnitude(v) gives length of v."""
def magnitude(v):
    v=np.asarray(v)
    return np.linalg.norm(v)

"""Lie Algebra Operations-----------------------------------------------------"""
 
"""Generate a skew symmetric matrix from an axis"""
"""synonym"""
def hat(w):
    return axisToSkew(w)

def skew(w):
    return axisToSkew(w)

def axisToSkew(omega): 
    #Check to make sure the dimensions are okay
    omega=np.asarray(omega)
    if omega.shape[0]==1:
        mat=np.array([[0,-omega[0,2],omega[0,1]],
                  [omega[0,2],0,-omega[0,0]],
                  [-omega[0,1],omega[0,0],0]])
        return mat
    else:
        raise Exception("FUNCTION axisToSkew: wrong dimension") 


"""skewToAxis(s) extracts vector w from skew-symmetric matrix s"""
def skewToAxis(s):
    s=np.asarray(s)
    if (s.shape[0]==s.shape[1]==3):
        sNew=np.array([[s[2,1]],[s[0,2]],[s[1,0]]])
        return sNew
    else:
        raise Exception("FUNCTION skewToAxis: wrong dimension")
        
def hatInv(s):               #synonyms
    return skewToAxis(s)
    
def unSkew(s):               #synonyms
    return skewToAxis(s)

def vee(s):                  #synonyms
    return skewToAxis(s)
  
"""skewExp[w,(theta)] gives the matrix exponential of an axis w.
  Default value of Theta is 1."""
def skewExp(s,theta):
    s=np.asarray(s)
    if (s.shape[0]==1):
        s=axisToSkew(s)
    sNew=eye(3) + np.sin(theta)*s + ((1-np.cos(theta))*(s.dot(s)))
    return sNew
"""
  Rigid transformation matrices
  Operations on se(3), the Lie algebra of rigid motions SE(3)
"""    

"""Figure out the dimension of a twist [private]"""
def xidim(xi):
    xi=np.asarray(xi)
    l=xi.shape[1]
    n=(np.sqrt(1+8*l)-1)/2
    return n

"""Extract the angular portion of a twist [private]"""
"""xitow(xi) gives the angular part of twist xi."""
def xitow(xi):
    xi=np.asarray(xi)
    n=xidim(xi)
    #Make sure that the vector had a reasonable length
    if (n==0):
        raise Exception("FUNCTION xitow: n=0")
    #Extract the angular portion of the twist *)
    #SetPrecision[Take[xi, -(n (n-1) / 2)],PRECISION]
    else:
        indexC=xi.shape[1]-(n*(n-1)/2)
        return takeMatrix(xi,0,indexC,0,xi.shape[1])
        
"""Extract the linear portion of a twist [private]"""
"""xitov(xi) gives the translational part of twist xi."""
def xitov(xi):
    xi=np.asarray(xi)
    n=xidim(xi)
    #Make sure that the vector had a reasonable length
    if (n==0):
        raise Exception("FUNCTION xitov: n=0")
    #Extract the angular portion of the twist *)
    #SetPrecision[Take[xi, n],PRECISION]
    else:
        return takeMatrix(xi,0,0,0,n-1)

"""Rotation Matrices - forward kin map---------------------------------------"""

"""rotX(alpha) gives the rotation matrix about the X axis."""
def rotX(alphaRad):
    ca=np.cos(alphaRad)
    sa=np.sin(alphaRad)
    mat=np.array([[1,0,0],
                  [0,ca,-sa],
                  [0,sa,ca]])
    return mat

"""rotY(alpha) gives the rotation matrix about the Y axis."""
def rotY(alphaRad):
    ca=np.cos(alphaRad)
    sa=np.sin(alphaRad)
    mat=np.array([[ca,0,sa],
                  [0,1,0],
                  [-sa,0,ca]])
    return mat

"""rotZ(alpha) gives the rotation matrix about the Z axis."""
def rotZ(alphaRad):
    ca=np.cos(alphaRad)
    sa=np.sin(alphaRad)
    mat=np.array([[ca,-sa,0],
                  [sa,ca,0],
                  [0,0,1]])
    return mat

"""aTan2(x, y) gives the angle with tangent given by y/x in the right quadrant."""
def aTan2(xRad, yRad):
    return np.arctan2(xRad,yRad)

"""eulZXZToMat(alpha, beta, gamma) returns the rotation matrix corresponding to the ZXZ Euler angles.
This is the one of the standard parametrizations employed in Multibody Dynamics."""
def eulZXZToMat(alpha, beta, gamma) :
    A=rotZ(alpha)
    B=rotX(beta)
    C=rotZ(gamma)
    return (A.dot(B)).dot(C)

"""eulZYZToMat(alpha, beta, gamma) returns the rotation matrix corresponding to the ZYZ Euler angles.
This is the one of the standard parametrizations employed in Robotics."""
def eulZYZToMat(alpha, beta, gamma) :
    A=rotZ(alpha)
    B=rotY(beta)
    C=rotZ(gamma)
    return (A.dot(B)).dot(C)

"""eulZYXToMat(alpha, beta, gamma) returns the rotation matrix corresponding to the ZYX Euler angles,
also known as Roll, Pitch and Yaw.
This is one of the standard parametrizations employed in Robotics."""
def eulZYXToMat(alpha, beta, gamma) :
    A=rotZ(alpha)
    B=rotY(beta)
    C=rotX(gamma)
    return (A.dot(B)).dot(C)

def RPYToMat(alpha, beta, gamma) :
    return eulZYXToMat(alpha, beta, gamma)

"""eulXYZToMat(alpha, beta, gamma) returns the rotation matrix corresponding to the XYZ Euler angles.
This is useful, accompanied by its corresponding spatial Jacobian for small rotations."""
def eulXYZToMat(alpha, beta, gamma) :
    A=rotX(alpha)
    B=rotY(beta)
    C=rotZ(gamma)
    return (A.dot(B)).dot(C)

"""rodriguezToMat(g1, g2, g3) returns the rotation matrix corresponding to the Rodriguez parameters g1, g2, g3.
Recall that (g1, g2, g3) = r tan(theta/2)."""
def rodriguezToMat(g1, g2, g3):
    gamma=np.array([g1,g2,g3])
    hatgamma=hat([gamma])
    modulusgammasquared=gamma.dot(gamma)
    mat= eye(3)+((2./(1 + modulusgammasquared))*(hatgamma + hatgamma.dot(hatgamma)))
    return mat

"""Rotation Matrices - forward differential kin map----------------------------"""

"""eulZXZToSpatialJac(alpha, beta, gamma) returns the Spatial Jacobian Js corresponding to the ZXZ Euler angles.
This is such that w_s = Js dot{alpha, beta, gamma}, with w_s spatial components."""
def eulZXZToSpatialJac(phiRad, thetaRad, psiRad):
    mat=np.array([[0,np.cos(phiRad),np.sin(phiRad)*np.sin(thetaRad)],
                  [0,np.sin(phiRad),-np.cos(phiRad)*np.sin(thetaRad)],
                  [1,0,np.cos(thetaRad)]])
    return mat

"""eulZXZToBodyJac(alpha, beta, gamma) returns the Body Jacobian Jb corresponding to the ZXZ Euler angles.
This is such that w_b = Jb dot{alpha, beta, gamma}, with w_b body fixed components."""
def eulZXZToBodyJac(phiRad, thetaRad, psiRad):
    mat=np.array([[np.sin(thetaRad)*np.sin(psiRad), np.cos(psiRad),0],
                  [np.sin(thetaRad)*np.cos(psiRad), -np.sin(psiRad),0],
                  [np.cos(thetaRad),0,1]])
    return mat

"""eulZYZToSpatialJac(alpha, beta, gamma) returns the Spatial Jacobian Js corresponding to the ZYZ Euler angles.
This is such that w_s = Js dot{alpha, beta, gamma}, with w_s spatial components."""
def eulZYZToSpatialJac(phiRad, thetaRad, psiRad):
    mat=np.array([[0,-np.sin(phiRad),np.cos(phiRad)*np.sin(thetaRad)],
                  [0,np.cos(phiRad),np.sin(phiRad)*np.sin(thetaRad)],
                  [1,0,np.cos(thetaRad)]])
    return mat

"""eulZYZToBodyJac(alpha, beta, gamma) returns the Body Jacobian Jb corresponding to the ZYZ Euler angles.
This is such that w_b = Jb dot{alpha, beta, gamma}, with w_b body fixed components."""
def eulZYZToBodyJac(phiRad, thetaRad, psiRad):
    mat=np.array([[-np.cos(psiRad)*np.sin(thetaRad),np.sin(psiRad),0],
                  [np.sin(psiRad)*np.sin(thetaRad),np.cos(psiRad),0],
                  [np.cos(thetaRad),0,1]])
    return mat

"""eulZYXToSpatialJac(alpha, beta, gamma) returns the Spatial Jacobian Js corresponding to the ZYX Euler angles.
This is such that w_s = Js dot{alpha, beta, gamma}, with w_s spatial components."""
def eulZYXToSpatialJac(phiRad, thetaRad, psiRad):
    mat=np.array([[0,-np.sin(phiRad),np.cos(thetaRad)*np.cos(phiRad)],
                  [0,np.cos(phiRad), np.cos(thetaRad)*np.sin(phiRad)],
                  [1,0,-np.sin(thetaRad)]])
    return mat

"""EulZYXToBodyJac[alpha, beta, gamma] returns the Body Jacobian Jb corresponding to the ZYX Euler angles.
This is such that w_b = Jb dot{alpha, beta, gamma}, with w_b body fixed components."""
def eulZYXToBodyJac(phiRad, thetaRad, psiRad):
    mat=np.array([[-np.sin(thetaRad),0,1],
                  [np.cos(thetaRad)*np.sin(psiRad),np.cos(psiRad),0],
                  [np.cos(thetaRad)*np.cos(psiRad),-np.sin(psiRad),0]])
    return mat   

"""eulXYZToSpatialJac(alpha, beta, gamma) returns the Spatial Jacobian Js corresponding to the XYZ Euler angles.
This is such that w_s = Js dot{alpha, beta, gamma}, with w_s spatial components."""
def eulXYZToSpatialJac(phiRad, thetaRad, psiRad):
    mat=np.array([[1,0,np.sin(thetaRad)],
                  [0,np.cos(phiRad),-np.cos(thetaRad)*np.sin(phiRad)],
                  [0,np.sin(phiRad),np.cos(thetaRad)*np.cos(phiRad)]])
    return mat 

"""quatToSpatialJac(q) returns the Spatial Jacobian Js corresponding to the unit quaternion q (1-vector).
If q = (q0, q1, q2, q3), w_s = Js dot{q}."""
def quatToSpatialJac(b0,b1,b2,b3):
    mat=np.array([[-b1,  b0, -b3,  b2],
                  [-b2,  b3,  b0, -b1],
                  [-b3, -b2,  b1,  b0]])
    return 2*mat

"""q is matrix 1x4"""
def quatToSpatialJacFromList(q):
    q=np.asarray(q)
    b0=q[0,0]
    b1=q[0,1]
    b2=q[0,2]
    b3=q[0,3]
    return quatToSpatialJac(b0,b1,b2,b3)

"""quatToBodyJac(q) returns the Body Jacobian Jb corresponding to the unit quaternion q (1-vector).
If q = (q0, q1, q2, q3), w_b = Jb dot{q}."""
def quatToBodyJac(b0,b1,b2,b3):
    mat=np.array([[-b1,  b0,  b3, -b2],
                  [-b2, -b3,  b0,  b1],
                  [-b3,  b2, -b1,  b0]])
    return 2*mat

"""q is matrix 1x4"""
def quatToBodyJacFromList(q):
    q=np.asarray(q)
    b0=q[0,0]
    b1=q[0,1]
    b2=q[0,2]
    b3=q[0,3]
    return quatToBodyJac(b0,b1,b2,b3)


"""rodriguezToSpatialJac(g1, g2, g3) returns the Spatial Jacobian Js corresponding to Rodriguez parameters.\
This is such that w_s = Js dot{g1, g2, g3}, with w_s spatial components."""
def rodriguezToSpatialJac(gamma1, gamma2, gamma3):
    gamma=np.array([gamma1,gamma2,gamma3])
    modulusgammasquared=gamma.dot(gamma)
    mat=np.array([[1,-gamma3,gamma2],
                  [gamma3,1,-gamma1],
                  [-gamma2,gamma1,1]])
    return (2./(1 + modulusgammasquared))*mat

"""rodriguezToBodyJac(g1, g2, g3) returns the Body Jacobian Jb corresponding to Rodriguez parameters.\
This is such that w_b = Jb dot{g1, g2, g3}, with w_b body fixed components."""
def rodriguezToBodyJac(gamma1, gamma2, gamma3):
    gamma=np.array([gamma1,gamma2,gamma3])
    modulusgammasquared=gamma.dot(gamma)
    mat=np.array([[1,gamma3,-gamma2],
                  [-gamma3,1,gamma1],
                  [gamma2,-gamma1,1]])
    return (2./(1 + modulusgammasquared))*mat


"""Rotation Matrices - inverse kin map-----------------------------------------"""

"""matToEulZYZ(R) returns the Euler angles for the parametrization ZYZ 
corresponding to the rotation matrix R."""
def matToEulZYZ(R):
    R=np.asarray(R)
    # Check the singularity of representation ZYZ
    #In cases of singularity we set arbitrarily psi = 0
    if (np.abs(R[2,2]-1) < 10**(-10)):
        phi = aTan2(R[1,0],R[0,0])
        theta = 0
        psi = 0
    elif(np.abs(R[2,2]+1) < 10**(-10)):
        phi = aTan2( R[1,0], R[0,0])
        theta = np.pi;
        psi = 0;
    else:
        phi = aTan2(R[1,2], R[0,2]);
        theta = aTan2(np.sqrt(R[0,2]**2 + R[1,2]**2), R[2,2]);
        psi = aTan2(R[2,1], -R[2,0]);
    return np.array([phi, theta, psi])

"""matToEulZYX(R) returns the Euler angles for the parametrization ZYX 
(also called RPY) corresponding to the rotation matrix R."""
def matToEulZYX(R):
    R=np.asarray(R)
    #Check the singularity of representation ZYX
    #In cases of singularity we set arbitrarily psi = 0
    if (np.abs(R[2,0]+1) < 10**(-10)):
        phi = aTan2(R[1,2],R[0,2])
        theta = np.pi/2
        psi = 0
    elif(np.abs(R[2,0]-1) < 10**(-10)):
        phi = aTan2(-R[1,2], R[0,2])
        theta = -np.pi/2;
        psi = 0;
    else:
        phi = aTan2(R[1,0], R[0,0]);
        theta = aTan2(-R[2,0],np.sqrt(R[2,1]**2 + R[2,2]**2));
        psi = aTan2(R[2,1], R[2,2]);
    return np.array([phi, theta, psi])

def matToRPY(R):
    return matToEulZYX(R)

"""framesTovect[{ R, Rd }] returns the orientation error e_O = r sin(theta) of 
frame Rd (desired) w.r.t. R (actual) in the spatial frame where both R and Rd are expressed."""
#TODO da testare
def framesTovect (list):
    list=np.asarray(list)
    R=list[0]
    Rd=list[1]
    hn=hat([R[:,0]])
    hs=hat([R[:,1]])
    ha=hat([R[:,2]])
    nd=Rd[:,0]
    sd=Rd[:,1]
    ad=Rd[:,2]
    eO=(1/2)*(hn.dot(nd) + hs.dot(sd) + ha.dot(ad) )
    return  eO

"""matToVect[R] or matToVect[ {R, Rd} ] returns the orientation error e_O = r sin(theta), 
where r and theta are the components of the axis angle parametrization.
e_O represents also the axis form of the skew-symm matrix R_SS = (1/2) (R - R^T)."""
def matToVect (list):
    return framesTovect(list)

"""Homogeneous Representations-----------------------------------------------"""

"""homogeneousToVector(q) extracts the cartesian c
oordinates from homogeneous components q."""
def homogeneousToVector (q, indexStart, indexFinish):
    a=np.asarray(q)
    return a[indexStart:(indexFinish+1)]
     
#Convert a point into homogeneous coordinates
"""pointToHomogeneous(q) gives the homogeneous representation of a point."""
def pointToHomogeneous (q):
    a=np.asarray(q)
    #Now put everything together into a homogeneous vector
    return np.append(a, 1)

#Convert a point into homogeneous coordinates
"""vectorToHomogeneous(q) gives the homogeneous representation of a vector."""
def vectorToHomogeneous (q):
    a=np.asarray(q)
    #Now put everything together into a homogeneous vector
    return np.append(a, 0)

"""RPToHomogeneous(R,p) forms homogeneous matrix from rotation matrix R
 and position vector p."""
#NOTE: p is defined as [[v1, v2, v3]] and not [v1,v2,v3]
#this definition is important to execute transpose function
def RPToHomogeneous (R,p):
    #Convert a rotation + a translation to a homogeneous matrix
    mat=np.asarray(R)
    vect=np.asarray(p)
    if (mat.shape[0]==mat.shape[1]==vect.shape[1]==3):
        vectT=np.transpose(vect)
        matNew=R
        matNew=stackCols(matNew, vectT)
        matNew=stackRows(matNew, [[0,0,0,1]])
        return matNew
    else:
        raise Exception("FUNCTION RPToHomogeneous: wrong dimension")
  
"""rigidOrientation(g) extracts rotation matrix R from g."""
def rigidOrientation(g):
    mat=np.asarray(g)
    nr=mat.shape[0]
    nc=mat.shape[1]
    if (nc==nr and nr>=3):
        return subMatrix(mat,0,0,nc-1,nc-1)
    else:
        raise Exception("FUNCTION rigidOrientation: wrong dimension") 

"""rigidPosition(g) extracts position vector p from g."""
def rigidPosition(g):
    mat=np.asarray(g)
    nr=mat.shape[0]
    nc=mat.shape[1]
    if (nc==nr and nr>=3):
        return subMatrix(mat,0,nc-1,nc-1,1)
    else:
        raise Exception("FUNCTION rigidPosition: wrong dimension") 
  
"""rigidInverse(g) gives the inverse transformation of g."""
def rigidInverse(g):
    mat=np.asarray(g)
    R=rigidOrientation(mat)
    p=rigidPosition(mat)
    RT=np.transpose(R)
    matNew=RPToHomogeneous(RT, np.transpose(-RT.dot(p)))
    return matNew

def nullspace(A, atol=1e-13, rtol=0):
    """Compute an approximate basis for the nullspace of A.

    The algorithm used by this function is based on the singular value
    decomposition of `A`.

    Parameters
    ----------
    A : ndarray
        A should be at most 2-D.  A 1-D array with length k will be treated
        as a 2-D with shape (1, k)
    atol : float
        The absolute tolerance for a zero singular value.  Singular values
        smaller than `atol` are considered to be zero.
    rtol : float
        The relative tolerance.  Singular values less than rtol*smax are
        considered to be zero, where smax is the largest singular value.

    If both `atol` and `rtol` are positive, the combined tolerance is the
    maximum of the two; that is::
        tol = max(atol, rtol * smax)
    Singular values smaller than `tol` are considered to be zero.

    Return value
    ------------
    ns : ndarray
        If `A` is an array with shape (m, k), then `ns` will be an array
        with shape (k, n), where n is the estimated dimension of the
        nullspace of `A`.  The columns of `ns` are a basis for the
        nullspace; each element in numpy.dot(A, ns) will be approximately
        zero.
    """

    A = np.atleast_2d(A)
    u, s, vh = svd(A)
    tol = max(atol, rtol * s[0])
    nnz = (s >= tol).sum()
    ns = vh[nnz:].conj().T
    return ns

"""rotationAxis(R,theta) returns the rotation axis of R in SO(3)."""  
#NOTE: theta is radiant
def rotationAxis(R,theta):
    mat=np.asarray(R)
    nr=mat.shape[0]
    nc=mat.shape[1]
    if (nc!=nr):
        raise Exception("FUNCTION rotationAxis: wrong dimension") 
    elif(theta<0 or theta>np.pi):
        raise Exception("FUNCTION rotationAxis: wrong theta")
    elif(theta==np.pi):
        temp=R-eye(3)
        axis=nullspace(temp);
        axis=axis/magnitude(axis);
        return axis
    else:
        axis=np.array([[R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1]]])
        axis=axis/(2*np.sin(theta))
        return axis
  
"""rotationParam[R] returns the rotation axis the and 
amount of rotation of R in SO(3)."""
def rotationParam(R):
    mat=np.asarray(R)
    nr=mat.shape[0]
    nc=mat.shape[1]
    if (nc==nr==3):
        t = ((R[0,0]+R[1,1]+R[2,2])-1)/2.
        if (t<-1):
            t=-1
        if (t>1):
            t=1
        theta=np.arccos(t)
        if (theta!=0):
            axis=rotationAxis(R, theta)
        else:
            axis=np.array([[0,0,0]])
            theta=0
    return np.array([np.append(axis, theta)])


"""Operations on se(e), the Lie algebra of SE(3)------------------------------"""

""" Convert a homogeneous matrix to a twist 
 This only works in dimensions 2 and 3 for now """
"""homogeneousToTwist(xi) converts xi from a 4x4 matrix to a 6 vector."""
def homogeneousToTwist(A):
    mat=np.asarray(A)
    nr=mat.shape[0]
    nc=mat.shape[1]
    if (nr!=nc):
        raise Exception("FUNCTION homogeneousToTwist: wrong dimension of A")
    #Make sure that we have a twist and not a rigid motion
    if (A[nr-1,nc-1]!=0):
        raise Exception("FUNCTION homogeneousToTwist: notTwistMatrix")
    #Extract the skew part and the vector part and make a vector
    mat1=subMatrix(A, 0, nr-1, nr - 1, 1)
    mat2=subMatrix(A, 0, 0, nr - 1 ,nc - 1)
    mat2=skewToAxis(mat2)
    return np.concatenate((mat1,mat2),axis=0)

"""Convert a twist to homogeneous coordinates"""
"""twistToHomogeneous(xi) converts xi from a 6 vector to a 4x4 matrix."""
def twistToHomogeneous(xi):
    xi=np.asarray(xi)
    w=xitow(xi)
    v=xitov(xi)
    #Now put everything together into a homogeneous transformation
    mat1=axisToSkew(w)
    mat2=np.transpose(v)
    matNew=stackCols(mat1, mat2)
    matNew=stackRows(matNew, [[0,0,0,0]])
    return matNew

"""Take the exponential of a twist
  This only works in dimension 3 for now !"""
"""twistExp(xi,Theta) gives the matrix exponential of a twist xi.
  Default value of Theta is 1."""
def twistExp (xi, theta):
    xi=np.asarray(xi)
    if (xi.shape[0]==3):
        xi=homogeneousToTwist(xi)
    w=xitow(xi)
    v=xitov(xi)
    v=np.transpose(v)
    #Use the exponential formula from MLS
    if (w[0,0]==0 and w[0,1]==0 and w[0,2]==0):
        R=eye(3)
        p=np.transpose(v*theta)
    else:
        ws=skew(w)
        R=skewExp(ws, theta)
        if (v[0,0]==0 and v[1,0]==0 and v[2,0]==0):
            mat1=np.array([[0,0,0]])
            mat2=np.array([[0,0,0]])
        else:
            mat1=np.transpose((eye(3)-R).dot(ws.dot(v)))
            mat2=(w*(w.dot(v))*theta)
        p=mat1+mat2
    return RPToHomogeneous(R,p)
  
"""rigidTwist(g) extracts 6 vector xi from g and the angle theta that generates g."""
def rigidTwist(g):
    #Extract the appropriate pieces of the homogeneous transformation
    R = rigidOrientation(g);
    p = rigidPosition(g);
    #Now find the axis from the rotation  
    #w = rotationAxis(R)
    #theta = rotationAngle(R)
    vect=rotationParam(R)
    lVect=vect.shape[1]
    theta=vect[0,lVect-1]
    w=np.array([vect[0,0:lVect-1]])
    hatw = hat(w);     
    #Split into cases depending on whether theta is zero
    if (theta==0):
        theta=magnitude(p)
        if (theta==0):
            return np.array([[0,0,0,0,0,0]])
        v=p/theta
    else:
        MP=LA.matrix_power(hatw,2)
        aInv=(eye(3)/theta -(1./2)*hatw)+ ((1./theta - (1./2)*(1./np.tan(theta/2)))*MP)
        v = np.transpose(p).dot(np.transpose(aInv))
    matNew=np.append(v, w)
    matNew=np.append(matNew,theta)
    return np.array([matNew])

"""screwToTwist(h, q, w) builds a twist from its screw 
coordinates h (pitch), q (point on axis), w (axis)."""
def screwToTwist (h,q,w):
    if (np.isinf(h)):
        matNew=np.append(w,[0,0,0])
    else:
        temp=-axisToSkew(w).dot(q) + h*w
        matNew=np.append(temp,w)
    return np.array([matNew])

"""twistPitch(xi) gives pitch of screw corresponding to a twist."""
def twistPitch(xi):
    #dimension xi is 1 rows and 6 columns
    xi=np.asarray(xi)
    v=np.array([xi[0,0:3]])
    w=np.array([xi[0,3:6]])
    #this operation v.dot(np.transpose(w)) return a matrix 1x1
    num=v.dot(np.transpose(w))[0,0]
    #this operation w.dot(np.transpose(w)) return a matrix 1x1
    den=w.dot(np.transpose(w))[0,0]
    return float(num)/den

"""twistAxis(xi) gives axis of screw corresponding to a 
twist as q (point on axis), w (axis)"""
def twistAxis(xi):
    xi=np.asarray(xi)
    v=np.array([xi[0,0:3]])
    w=np.array([xi[0,3:6]])
    if (w[0,0]==w[0,1]==w[0,2]==0):
        temp=v/(np.sqrt(v.dot(np.transpose(v))))
        matNew=stackRows([[0,0,0]], temp)
    else:
        den=w.dot(np.transpose(w))[0,0]
        temp1=axisToSkew(w).dot(np.transpose(v)) / den
        temp2=w/den
        matNew=stackRows(np.transpose(temp1), temp2)
    return matNew

"""twistMagnitude(xi) gives magnitude of screw corresponding to a twist."""
def twistMagnitude(xi):
    xi=np.asarray(xi)
    v=np.array([xi[0,0:3]])
    w=np.array([xi[0,3:6]])
    if (w[0,0]==w[0,1]==w[0,2]==0):
        value=np.sqrt(v.dot(np.transpose(v))[0,0])
    else:
        value=np.sqrt(w.dot(np.transpose(w))[0,0])
    return value

"""Adjoint matrix calculations-------------------------------------------------"""

"""rigidAdjoint(g) / adRA(g) gives the Adjoint matrix corresponding to g."""
def rigidAdjoint(g):
    R=rigidOrientation(g)
    p=rigidPosition(g)
    p=np.transpose(p)
    temp1=axisToSkew(p).dot(R)
    temp1=stackCols(R,temp1)
    temp2=zeroMatrix(3)
    temp2=stackCols(temp2, R)
    newMat=stackRows(temp1, temp2)
    return newMat

def adRA(g):
    return rigidAdjoint(g) 

"""inverseRigidAdjoint(g) / adInvRA(g) gives the inverse Adjoint matrix corresponding to g."""  
def inverseRigidAdjoint(g):
    R=rigidOrientation(g)
    RT=np.transpose(R)
    p=rigidPosition(g)
    p=np.transpose(p)
    temp1=-RT.dot(axisToSkew(p))
    temp1=stackCols(RT,temp1)
    temp2=zeroMatrix(3)
    temp2=stackCols(temp2, RT)
    newMat=stackRows(temp1, temp2)
    return newMat

def adInvRA(g):
    return inverseRigidAdjoint(g) 

"""transposeRigidAdjoint(g) / adTrRA(g) gives the transpose Adjoint matrix corresponding to g."""  
def transposeRigidAdjoint(g):
    R=rigidOrientation(g)
    RT=np.transpose(R)
    p=rigidPosition(g)
    p=np.transpose(p)
    temp1=zeroMatrix(3)
    temp1=stackCols(RT, temp1)
    temp2=-RT.dot(axisToSkew(p))
    temp2=stackCols(temp2,RT)
    newMat=stackRows(temp1, temp2)
    return newMat

def adTrRA(g):
    return transposeRigidAdjoint(g) 

"""inverseTransposeRigidAdjoint(g) / adStarRA(g) gives the inverse 
transpose Adjoint matrix corresponding to g."""
def inverseTransposeRigidAdjoint(g):
    R=rigidOrientation(g)
    p=rigidPosition(g)
    p=np.transpose(p)
    temp1=zeroMatrix(3)
    temp1=stackCols(R, temp1)
    temp2=axisToSkew(p).dot(R)
    temp2=stackCols(temp2,R)
    newMat=stackRows(temp1, temp2)
    return newMat

def adStarRA(g):
    return inverseTransposeRigidAdjoint(g) 


"""adjoint (small) matrix calculations - aka Lie Bracket----------------------"""


"""ad(xi) gives the adjoint matrix associated to twist xi."""
def ad(xi):
    xi=np.asarray(xi)
    v=np.array([xi[0,0:3]])
    w=np.array([xi[0,3:6]])
    vs=hat(v)
    ws=hat(w)
    temp1=stackCols(ws,vs)
    temp2=zeroMatrix(3)
    temp2=stackCols(temp2, ws)
    newMat=stackRows(temp1, temp2)
    return newMat
  
"""adTr(xi) gives the transpose adjoint matrix associated to xi."""
def adTr(xi):
    xi=np.asarray(xi)
    v=np.array([xi[0,0:3]])
    w=np.array([xi[0,3:6]])
    vs=hat(v)
    ws=hat(w)
    temp1=zeroMatrix(3)
    temp1=stackCols(-ws,temp1)
    temp2=stackCols(-vs,-ws)
    newMat=stackRows(temp1, temp2)
    return newMat
  
"""adStar(xi) gives the negative transpose adjoint matrix associated to xi."""   
def adStar(xi):
    xi=np.asarray(xi)
    v=np.array([xi[0,0:3]])
    w=np.array([xi[0,3:6]])
    vs=hat(v)
    ws=hat(w)
    temp1=zeroMatrix(3)
    temp1=stackCols(ws,temp1)
    temp2=stackCols(vs,ws)
    newMat=stackRows(temp1, temp2)
    return newMat

"""lieBracket(xia, xib) / lieBracket(xia^, xib^) gives vee((xia^ xib^ - xib^ xia^))."""
def lieBracketDull(xia, xib):
    xias=twistToHomogeneous(xia)
    xibs=twistToHomogeneous(xib)
    xics=xias.dot(xibs) - xibs.dot(xias)
    return homogeneousToTwist(xics)

def lieBracket(xia, xib):
    xia=np.asarray(xia)
    xib=np.asarray(xib)
    if (xia.shape[0]==xia.shape[1] and  xib.shape[0]==xib.shape[1]):
        xia=homogeneousToTwist(xia)
        xib=homogeneousToTwist(xib)
        xia=np.transpose(xia)
        xib=np.transpose(xib)
    adxia=ad(xia)
    xib=np.transpose(xib)
    return adxia.dot(xib)
    
"""Operations on SO(n), the Lie group of rotation matrices--------------------"""

"""localTranTwist(gStart, gEnd) returns the twist and theta such that: 
gStart.TwistExp(xi,theta) = gEnd.
The xi coordinate are expressed in the local (gStart) reference frame."""
def localTranTwist (gStart, gEnd):
    gStart=np.asarray(gStart)
    gEnd=np.asarray(gEnd)
    gError = (LA.inv(gStart)).dot(gEnd)
    if ((gError==eye(4)).all()):
        return np.array([[0,0,0,0,0,0,0]])
    else:
        return rigidTwist(gError)

"""globalTranTwist(gStart, gEnd) returns the twist and theta such that: 
twistExp(xi,theta).gStart = gEnd.
The xi coordinate are expressed in the global (identity) reference frame."""
def globalTranTwist (gStart, gEnd):
    gStart=np.asarray(gStart)
    gEnd=np.asarray(gEnd)
    gError = (LA.inv(gStart)).dot(gEnd)
    if ((gError==eye(4)).all()):
        return np.array([[0,0,0,0,0,0,0]])
    else:
        temp=rigidTwist(gError)
        xi=np.array([temp[0,0:temp.shape[1]-1]])
        theta=np.array([temp[0,temp.shape[1]-1]])
        Adg=rigidAdjoint(gStart)
        xi=np.transpose(xi)
        xi=Adg.dot(xi)
        matNew=np.append(xi,theta)
        return np.array([matNew])

"""Systematic Methods to Build Forward Kinematics and Spatial/Body Jacobians---"""

"""revoluteTwist(q, w) builds the 6-vector corresponding to point q 
on the axis with unit vector w for a revolute joint."""
#Gives Xi 6 vector given a point on axis and axis unit vector for a Revolute Joint 
def revoluteTwist(q, w):
    q=np.asarray(q)
    w=np.asarray(w)
    temp=np.cross(q,w)
    newMat=np.append(temp,w)
    return np.array([newMat])

#Gives Xi 6 vector given a point on axis and axis unit vector for a Prismatic Joint
def prismaticTwist(q, w):
    q=np.asarray(q)
    w=np.asarray(w)
    newMat=np.append(w,[[0,0,0]])
    return np.array([newMat])

"""forwardKinematics(xi,th,g0) computes the forward 
kinematics via the product of exponentials formula.
 g0 is the initial affine tranformation if any."""
#th columns matrix nx1
def forwardKinematics(xi,th,g0):
    xi=np.asarray(xi)
    th=np.asarray(th)
    g0=np.asarray(g0)
    n=xi.shape[0]
    #Initialize the transformation matrix
    g=twistExp([xi[0,:]],th[0,0]) 
    #Build up the transformation matrix joint by joint
    for i in range(1,n):
        g=g.dot(twistExp([xi[i,:]],th[i,0]) )
    return g.dot(g0)


"""forwardKinematicsLocalPOE(M,X,q) computes the forward 
kinematics via the Local Product of Exponentials formula.
The outcome is g = M1.exp(X1*q1) ... Mn.exp(Xn*qn)."""
#M is matrix of matrix and is columns matrix nx1
#q columns matrix nx1
def forwardKinematicsLocalPOE(M,X,q):
    M=np.asarray(M)
    X=np.asarray(X)
    q=np.asarray(q)
    n=M.shape[0]
    #Initialize the transformation matrix
    g=M[0,0].dot(twistExp([X[0,:]],q[0,0]))
    #Build up the transformation matrix joint by joint
    for i in range(1,n):
        g=g.dot(M[i,0].dot(twistExp([X[i,:]],q[i,0])))
    return g

"""forwardKinematicsLocalPOETranslated(M,X,q) computes 
the forward kinematics via the Local Product of Exponentials formula.
The outcome is g = M1.exp(X1*q1) ... Mn.exp(Xn*qn). 
Please note that this result is computed by initially calculating the 
Global version of the twists and then applying a formula similar to the one 
used in ForwardKinematics."""
#M is matrix of matrix and is columns matrix nx1
#q columns matrix nx1
def forwardKinematicsLocalPOETranslated(M,X,q):
    M=np.asarray(M)
    X=np.asarray(X)
    q=np.asarray(q)
    n=M.shape[0]
    #Initialize the transformation matrix
    tempM=M[0,0]
    Y = adRA(tempM).dot(np.transpose([X[0,:]]))
    Y=np.transpose(Y)
    g = twistExp( Y , q[0,0] )
    #Build up the transformation matrix joint by joint *)
    for i in range(1,n):
        tempM=tempM.dot(M[i,0])
        Y = adRA(tempM).dot(np.transpose([X[i,:]]))
        Y=np.transpose(Y)
        g = g.dot(twistExp( Y , q[i,0] ))
    return g.dot(tempM)
    

"""spatialJacobian(xi,th,g0) computes the 
Spatial Manipulator Jacobian of a robot defined by the given twists."""
def spatialJacobian(xi,th,g0):
    xi=np.asarray(xi)
    th=np.asarray(th)
    g0=np.asarray(g0)
    n=xi.shape[0]
    # First initialize the Jacobian and compute the first column
    Js = np.array([xi[0,:]])
    Js=np.transpose(Js)
    g=twistExp([xi[0,:]],th[0,0]) 
    #Build up the Jacobian joint by joint
    for i in range(1,n):
        #Compute this column of the Jacobian and append it to Js
        xiTemp= rigidAdjoint(g).dot(np.transpose([xi[i,:]]))
        Js=stackCols(Js,xiTemp)    
        #Update the transformation matrix
        g = g.dot(twistExp([xi[i,:]],th[i,0]))     
    #Return the Jacobian *)
    return Js
    
"""bodyJacobian(xi,th,g0) computes the 
Body Manipulator Jacobian of a robot defined by the given twists."""
def bodyJacobian(xi,th,g0):
    xi=np.asarray(xi)
    th=np.asarray(th)
    g0=np.asarray(g0)
    n=xi.shape[0]
    Jb=np.array([[]])
    g=g0
    #Build up the Jacobian joint by joint
    for i in range(1,n+1):
        #Compute this column of the Jacobian and prepend it to Jb 
        xiTemp=rigidAdjoint(rigidInverse(g)).dot(np.transpose([xi[n-i,:]]))
        if(i==1):
            Jb=xiTemp
        else:
            Jb=stackCols(xiTemp,Jb)
        #Update the transformation matrix
        g = twistExp([xi[n-i,:]], th[n-i,0]).dot(g);
    return Jb

"""spatialJacobianLocalPOE(M,X,q) computes the 
Spatial Manipulator Jacobian of a robot defined by the given twists
described in the Local Product of Exponentials form."""
def spatialJacobianLocalPOE(M,X,q):
    M=np.asarray(M)
    X=np.asarray(X)
    q=np.asarray(q)
    n=M.shape[0]
    g=eye(4)
    #Build up the Jacobian joint by joint
    for i in range(0,n):
        #Update the transformation matrix
        g = g.dot( M[i,0].dot( twistExp([X[i,:]], q[i,0] )))
        #Compute this column of the Jacobian and append it to Js *)
        xi= [adRA(g).dot(X[i,:])]
        xi=np.transpose(xi)
        if(i==0):
            Js=xi
        else:
            Js=stackCols(Js,xi)
    return Js

"""bodyJacobianLocalPOE(M,X,q) computes the 
Body Manipulator Jacobian of a robot defined by the given twists
described in the Local Product of Exponentials form."""
def bodyJacobianLocalPOE(M,X,q):
    M=np.asarray(M)
    X=np.asarray(X)
    q=np.asarray(q)
    n=M.shape[0]
    g=eye(4)
    #Build up the Jacobian joint by joint
    for i in range(1,n+1):
        xi=adInvRA(g).dot(np.transpose([X[n-i,:]]))
        if(i==1):
            Jb=xi
        else:
            Jb=stackCols(xi,Jb) 
        #Update the transformation matrix
        g = M[n-i,0].dot(twistExp([X[n-i,:]], q[n-i,0])).dot(g)
    return Jb



""""forwardKinematicsLeft(xi,th,g0) computes the forward 
kinematics via the product of exponentials formula, premultiplicating the offset matrix.
g0 is the initial affine tranformation if any."""
def forwardKinematicsLeft(xi,th,g0):
    xi=np.asarray(xi)
    th=np.asarray(th)
    g0=np.asarray(g0)
    n=xi.shape[0]
    g=g0.dot(twistExp([xi[0,:]],th[0,0]) )
    for i in range(1,n):
        g=g.dot(twistExp([xi[i,:]],th[i,0]) )
    return g

"""forwardKinematicsLocalPOERight(M,X,q) computes 
the forward kinematics via the Local Product of Exponentials formula.
Each offset matrix is post-multiplicated.
The outcome is g = exp(X1*q1).M1 ... exp(Xn*qn).Mn ."""
def forwardKinematicsLocalPOERight(M,X,q):
    M=np.asarray(M)
    X=np.asarray(X)
    q=np.asarray(q)
    n=M.shape[0]
    #Initialize the transformation matrix
    g=(twistExp([X[0,:]],q[0,0])).dot(M[0,0])
    #Build up the transformation matrix joint by joint
    for i in range(1,n):
        g=g.dot((twistExp([X[i,:]],q[i,0])).dot(M[i,0]))
    return g
 
 
"""forwardKinematicsDH(theta,d,alpha,a)
computes the forward kinematics via the Denavit-Hartenberg formula."""
def forwardKinematicsDH(theta,d,alpha,a):
    theta=np.asarray(theta)
    d=np.asarray(d)
    alpha=np.asarray(alpha)
    a=np.asarray(a)
    n=theta.shape[0]
    #Initialize the transformation matrix
    temp1=[ [0,0,d[0,0]] ]
    temp2=[ [a[0,0],0,0] ]
    g=RPToHomogeneous(rotZ(theta[0,0]),temp1).dot(RPToHomogeneous(rotX(alpha[0,0]),temp2))
    for i in range(1,n):
        temp1=[ [0,0,d[i,0]] ]
        temp2=[ [a[i,0],0,0] ]
        g=g.dot(RPToHomogeneous(rotZ(theta[i,0]),temp1).dot(RPToHomogeneous(rotX(alpha[i,0]),temp2)))
    #Return the final transformation
    return g

"""bodyJacobianGlobalPOE(xi,th,g0) computes the 
Body Manipulator Jacobian of a robot defined by the given twists."""
def bodyJacobianGlobalPOE(xi,th,g0):
    return bodyJacobian(xi,th,g0)

"""spatialJacobianGlobalPOE(xi,th,g0) computes the 
Spatial Manipulator Jacobian of a robot defined by the given twists
described in the Global POE form."""
def spatialJacobianGlobalPOE(xi,th,g0):
    return spatialJacobian(xi,th,g0) 
 
"""spatialJacobianGlobalPOELeft(xi,th,g0)
computes the Spatial Manipulator Jacobian of a robot defined by the Global POE Left form."""
def spatialJacobianGlobalPOELeft(xi,th,g0):
    xi=np.asarray(xi)
    th=np.asarray(th)
    g=np.asarray(g0)
    n=xi.shape[0]
    for i in range(0,n):
        g = g.dot(twistExp([xi[i,:]],th[i,0]))
        xiTemp=adRA(g).dot(np.transpose([xi[i,:]]))
        if i==0:
            Js=xiTemp
        else:
            Js=stackCols(Js,xiTemp)
    return Js 
 
"""bodyJacobianGlobalPOELeft(xi,th,g0) computes the 
Body Manipulator Jacobian of a robot defined by the Global POE Left form."""
def bodyJacobianGlobalPOELeft(xi,th,g0):
    xi=np.asarray(xi)
    th=np.asarray(th)
    g=eye(4)
    n=xi.shape[0]
    for i in range(0,n):
        g = (twistExp([xi[n-i-1,:]],th[n-i-1,0])).dot(g)
        xiTemp=inverseRigidAdjoint(g).dot(np.transpose([xi[n-i-1,:]]))
        if i==0:
            Jb=xiTemp
        else:
            Jb=stackCols(xiTemp,Jb)
    return Jb
 
"""spatialJacobianLocalPOERight(M,X,q) computes 
the Spatial Manipulator Jacobian of a robot defined by the given twists
described in the Local POE Right form."""
def spatialJacobianLocalPOERight(M,X,q):
    M=np.asarray(M)
    X=np.asarray(X)
    q=np.asarray(q)
    n=M.shape[0]
    g=eye(4)
    for i in range(0,n):
        xi=np.transpose([adRA(g).dot(X[i,:])])
        if i==0:
            Js=xi
        else:
            Js=stackCols(Js,xi)
        g=g.dot((twistExp([X[i,:]],q[i,0])).dot(M[i,0]))
    return Js 


"""bodyJacobianLocalPOERight(M,X,q) computes 
the Body Manipulator Jacobian of a robot defined by the given twists
described in the Local POE Right form."""
def bodyJacobianLocalPOERight(M,X,q):
    M=np.asarray(M)
    X=np.asarray(X)
    q=np.asarray(q)
    n=M.shape[0]
    g=eye(4)
    for i in range(0,n):
        g=(twistExp([X[n-i-1,:]],q[n-i-1,0])).dot(M[n-i-1,0]).dot(g)
        xi=np.transpose([inverseRigidAdjoint(g).dot(X[n-i-1,:])])
        if i==0:
            Jb=xi
        else:
            Jb=stackCols(xi,Jb)
    return Jb

""""spatialTwistDerivativeLocal(M,X,q,qp,qpp) 
computes the spatial derivative of a Twist given the twists
expressed in the Local frame, the joint angles,
 joint velocities and joint accelerations."""
def spatialTwistDerivativeLocal(M,X,q,qp,qpp):
    M=np.asarray(M)
    X=np.asarray(X)
    q=np.asarray(q)
    qp=np.asarray(qp)
    qpp=np.asarray(qpp)
    n=M.shape[0]
    #First initialize the quantities and the transformations
    gi=M[0,0].dot(twistExp([X[0,:]], q[0,0]))
    vabp=adRA(gi).dot(np.transpose([X[0,:]]))*qpp[0,0]
    #Build up Vabp joint by joint and by joint dependencies
    for i in range(1,n):
        gk=eye(4)
        #Compute the i-th column of the Jacobian
        xir=[X[i,:]]
        qi=q[i,0]
        qip=qp[i,0]
        qipp=qpp[i,0]
        #Update the transformation matrix in the outer cycle
        gi=gi.dot(M[i,0].dot(twistExp(xir, qi)))
        xi=adRA(gi).dot(np.transpose(xir))
        vabp=vabp+ xi*qipp
        for k in range(0,i):
            xkr=[X[k,:]]
            qk=q[k,0]
            qkp=qp[k,0]
            #Update the transformation matrix in the inner cycle
            gk=gk.dot(M[k,0].dot(twistExp(xkr, qk)))
            xk=adRA(gk).dot(np.transpose(xkr))
            vabp=vabp+ ad(np.transpose(xk)).dot(xi)*qkp*qip
    return vabp

"""bodyTwistDerivative(X,q,qp,qpp,gst0)
computes the derivative, expressed in the Body fixed reference frame, 
of a Spatial Twist given the twists
expressed in the Global frame, the joint angles, 
joint velocities and joint accelerations."""
def bodyTwistDerivative(X,q,qp,qpp,gst0):
    X=np.asarray(X)
    q=np.asarray(q)
    qp=np.asarray(qp)
    qpp=np.asarray(qpp)
    #First initialize the quantities and the transformations
    gi=np.asarray(gst0)
    n=X.shape[0]
    vabp=np.array([[0],[0],[0],[0],[0],[0]])  
    #Build up Vabp joint by joint and by joint dependencies
    for i in range(0,n):
        gk=np.asarray(gst0)
        #Compute the i-th column of the Jacobian
        xir=[X[n-i-1,:]]
        qi=q[n-i-1,0]
        qip=qp[n-i-1,0]
        qipp=qpp[n-i-1,0]
        xi=inverseRigidAdjoint(gi).dot(np.transpose(xir))
        vabp=vabp+ xi*qipp
        for k in range(0,i):
            xkr=[X[n-k-1,:]]
            qk=q[n-k-1,0]
            qkp=qp[n-k-1,0]
            xk=inverseRigidAdjoint(gk).dot(np.transpose(xkr))
            vabp=vabp-ad(np.transpose(xk)).dot(xi)*qkp*qip
            gk=twistExp(xkr, qk).dot(gk)
        # Update the transformation matrix in the outer cycle *)
        gi=twistExp(xir, qi).dot(gi)    
    return vabp

"""bodyTwistDerivativeLocal(M,X,q,qp,qpp)  
computes the derivative, expressed in the Body fixed reference frame, 
of a Twist given the twists
expressed in the Local frame, the joint angles, 
joint velocities and joint accelerations."""
def bodyTwistDerivativeLocal(M,X,q,qp,qpp):
    M=np.asarray(M)
    X=np.asarray(X)
    q=np.asarray(q)
    qp=np.asarray(qp)
    qpp=np.asarray(qpp)
    #First initialize the quantities and the transformations
    gi=eye(4)
    n=M.shape[0]
    vabp=np.array([[0],[0],[0],[0],[0],[0]])  
    #Build up Vabp joint by joint and by joint dependencies
    for i in range(0,n):
        gk=eye(4)
        #Compute the i-th column of the Jacobian
        xir=[X[n-i-1,:]]
        qi=q[n-i-1,0]
        qip=qp[n-i-1,0]
        qipp=qpp[n-i-1,0]
        xi=inverseRigidAdjoint(gi).dot(np.transpose(xir))
        vabp=vabp+xi*qipp
        for k in range(0,i):
            xkr=[X[n-k-1,:]]
            qk=q[n-k-1,0]
            qkp=qp[n-k-1,0]
            xk=inverseRigidAdjoint(gk).dot(np.transpose(xkr))
            vabp=vabp-ad(np.transpose(xk)).dot(xi)*qkp*qip
            #Update the transformation matrix in the inner cycle
            gk=M[n-k-1,0].dot(twistExp(xkr, qk).dot(gk))
        # Update the transformation matrix in the outer cycle *)
        gi=M[n-i-1,0].dot(twistExp(xir, qi).dot(gi))   
    return vabp
 
"""spatialTwistDerivative(X,q,qp,qpp,gst0)
computes the derivative of a Spatial Twist given the twists
expressed in the Global frame, the joint angles, 
joint velocities and joint accelerations."""
def spatialTwistDerivative(X,q,qp,qpp,gst0):
    X=np.asarray(X)
    q=np.asarray(q)
    qp=np.asarray(qp)
    qpp=np.asarray(qpp)
    gst0=np.asarray(gst0)
    n=X.shape[0]
    #First initialize the quantities and the transformations
    vabp=X[0,:]*qpp[0,0]
    vabp=np.transpose([vabp])
    gi=twistExp([X[0,:]],q[0,0])
    #Build up Vabp joint by joint and by joint dependencies
    for i in range(1,n):
        gk=eye(4)
        #Compute the i-th column of the Jacobian
        xir=[X[i,:]]
        xi=adRA(gi).dot(np.transpose(xir))
        qi=q[i,0]
        qip=qp[i,0]
        qipp=qpp[i,0]
        vabp=vabp+ xi*qipp
        for k in range(0,i):
            xkr=[X[k,:]]
            xk=adRA(gk).dot(np.transpose(xkr))
            qk=q[k,0]
            qkp=qp[k,0]
            vabp=vabp+ ad(np.transpose(xk)).dot(xi)*qkp*qip
            #Update the transformation matrix in the inner cycle
            gk=gk.dot(twistExp(xkr, qk))
        gi=gi.dot(twistExp(xir, qi))
    return vabp


"""quatToMat(q) returns the rotation matrix corresponding 
to the unit quaternion q (1-vector)."""
def quatToMat(b0,b1,b2,b3):
    mat=np.array([[b0**2+b1**2-b2**2-b3**2,  2*(b1*b2-b0*b3), 2*(b0*b2+b1*b3)],
                  [2*(b1*b2-b0*b3), b0**2-b1**2+b2**2-b3**2,  2*(b2*b3-b0*b1)],
                  [2*(b1*b3-b0*b2), 2*(b0*b1+b2*b3),  b0**2-b1**2-b2**2+b3**2]])
    return mat

"""q is matrix 1x4"""
def quatToMatFromList(q):
    q=np.asarray(q)
    b0=q[0,0]
    b1=q[0,1]
    b2=q[0,2]
    b3=q[0,3]
    return quatToMat(b0,b1,b2,b3)
