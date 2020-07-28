import numpy as np
import scipy.linalg as la

def homogenize(x):
    return np.append(x,1)

def decompose(P):
    '''
    Decomposes a camera projection matrix P into K, R, and t
    '''
    M = P[:3,:3]
    Minv = la.inv(M)
    C = -Minv @ P[:3,3]
    
    # R must be orthogonal
    # K must be upper triangular
    K,R = la.rq( M )
    
    # Enforce that K has positive diagonals
    T = np.diag( np.sign( np.diag(K) ) )
    K = K @ T
    K = K / K[2,2]
    R = T @ R
    
    t = -R @ C
    
    return K,R,t

def reconstruct(K,R,t):
    '''
    Reconstructs a camera matrix P from
    intrinsic matrix K
    rotation matrix R
    translation vector t
    used for debugging `decompose`
    '''
    Rt = np.vstack((R.T,t)).T
    P = K @ Rt
    return P

def triangulate_l2(u1,u2,P1,P2):
    K,R1,t1 = decompose(P1)
    _,R2,t2 = decompose(P2)

    # Relative rotation and translation that transform
    # a point from C1 to C2
    R = R2 @ R1.T
    t = -R2 @ R1.T @ t1 + t2
    
    # precompute for speedup
    Kinv = la.inv(K)
    
    m1 = R @ Kinv @ u1
    m2 = Kinv @ u2
    
    # unit vectors
    m1hat = m1 / la.norm(m1)
    m2hat = m2 / la.norm(m2)
    that = t / la.norm(t)
    
    # SVD matrix
    MT = np.vstack( (m1,m2) )
    P = np.eye(3) - np.outer(that,that)
    A = MT @ P
    _,_,Vh = la.svd(A)
    nhat = Vh[1]
    
    # corrections of m1 and m2
    m1p = m1 - np.dot(m1, nhat) * nhat
    m2p = m2 - np.dot(m2, nhat) * nhat
    
    Rf1p = m1p
    f2p = m2p
    z = np.cross(f2p, Rf1p)
    
    X1 = t + np.dot( z, np.cross(t, f2p) ) / la.norm(z)**2 * Rf1p
    X2 = np.dot( z, np.cross(t, Rf1p) ) / la.norm(z)**2 * f2p
    return R2.T @ (X2-t2)

def triangulate_linf(u1,u2,P1,P2):
    K,R1,t1 = decompose(P1)
    _,R2,t2 = decompose(P2)

    # Relative rotation and translation that transform
    # a point from C1 to C2
    R = R2 @ R1.T
    t = -R2 @ R1.T @ t1 + t2
    
    # precompute for speedup
    Kinv = la.inv(K)
    
    m1 = R @ Kinv @ u1
    m2 = Kinv @ u2
    
    # unit vectors
    m1hat = m1 / la.norm(m1)
    m2hat = m2 / la.norm(m2)
    
    # normals
    na = np.cross( ( m1hat + m2hat ), t )
    nb = np.cross( ( m1hat - m2hat ), t )
    n = na if la.norm(na) >= la.norm(nb) else nb
    nhat = n / la.norm(n)
    
    # corrections of m1 and m2
    m1p = m1 - np.dot(m1, nhat) * nhat
    m2p = m2 - np.dot(m2, nhat) * nhat
    
    Rf1p = m1p
    f2p = m2p
    z = np.cross(f2p, Rf1p)
    
    X1 = t + np.dot( z, np.cross(t, f2p) ) / la.norm(z)**2 * Rf1p
    X2 = np.dot( z, np.cross(t, Rf1p) ) / la.norm(z)**2 * f2p
    return R2.T @ (X2-t2)

def triangulate_l1(u1,u2,P1,P2):
    K,R1,t1 = decompose(P1)
    _,R2,t2 = decompose(P2)
    C1 = -R1.T @ t1
    C2 = -R2.T @ t2
    

    # Relative rotation and translation that transform
    # a point from C1 to C2
    R = R2 @ R1.T
    t = -R2 @ R1.T @ t1 + t2
    
    # precompute for speedup
    Kinv = la.inv(K)
    
    f1 = Kinv @ u1
    f2 = Kinv @ u2
    
    m1 = R @ f1
    m2 = f2
    
    # unit vectors
    m1hat = m1 / la.norm(m1)
    m2hat = m2 / la.norm(m2)
    
    # normals
    n1 = np.cross(m1, t)
    n2 = np.cross(m2, t)
    n1hat = n1 / la.norm(n1)
    n2hat = n2 / la.norm(n2)
    
    # corrections of m1 and m2
    if la.norm( np.cross(m1, t) ) <= la.norm( np.cross(m2, t) ):
        m1p = m1 - np.dot(m1, n2hat) * n2hat
        m2p = m2
    else:
        m1p = m1
        m2p = m2 - np.dot(m2, n1hat) * n1hat
    
    Rf1p = m1
    f2p = m2
    z = np.cross(f2p, Rf1p)
    
    X1 = t + np.dot( z, np.cross(t, f2p) ) / la.norm(z)**2 * Rf1p
    X2 = np.dot( z, np.cross(t, Rf1p) ) / la.norm(z)**2 * f2p
    return R2.T @ (X2-t2)