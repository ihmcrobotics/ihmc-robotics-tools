package us.ihmc.math.linearAlgebra.careSolvers.signFunction;

import org.ejml.data.DMatrixRMaj;

/**
 * If an nxn matrix K, has a Jordan canconical form
 * K = MJMinv = M(D + N) Minv
 * <p>
 * Then the matrix sign function of K is defined as
 * sign(K) = MSMinv = W
 * where S is a diagonal matrix whose
 * entries are given by
 * 1 if Re(d) &gt; 0 and -1 if Re(d) &lt; 0
 * <p>
 * If one of the eigenvalues of K lies on the imaginary axis,
 * sign(K) is undefined.
 */
public interface SignFunction
{
   boolean compute(DMatrixRMaj K);

   DMatrixRMaj getW(DMatrixRMaj WToPack);
}
