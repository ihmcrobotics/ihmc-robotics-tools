package us.ihmc.math.linearAlgebra.cdreSolvers;

import org.ejml.data.DMatrixRMaj;

/**
 * This solver computes the solution to the differential Riccati equation
 * <p>
 * A' P + P A - P B R^-1 B' P + Q = PDot
 * <p>
 * or
 * <p>
 * A' P + P A - P M P + Q = PDot
 * <p>
 * Or the more complex one
 * <p>
 * A' P E + E' P A - (B' P E + S' C)' R^-1 (B' P E + S' C) + C' Q C = PDot
 * <p>
 * or
 * <p>
 * A*' P E + E' P A* - E' P M P E + Q* = PDot
 * where
 * A* = A - B R^-1 S' C
 * Q* = C' Q C - C' S R^-1 S' C
 * <p>
 * where
 * <ul>
 *    <li>P is the unknown to be solved for</li>
 *    <li>R is the symmetric positive definite cost on the control.</li>
 *    <li>Q is the symmetric positive semi-definite cost on the output.</li>
 *    <li>S is the cross-cost on the control and state</li>
 *    <li>E is the output matrix</li>
 *    <li>A is the state transition matrix</li>
 *    <li>B is the control matrix.</li>
 *    <li>C is the output matrix,</li>
 * </ul>
 * I.e., we're solving the system
 * J = y' Q y + u' R u' + u' S' y + y' S u
 * E x = A x + B u
 * y = C x
 * <p>
 * It can be seen that, in the simpler system above, S = 0, E = C = I
 * <p>
 * We can also
 */
public interface CDRESolver
{
   void setMatrices(DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj C, DMatrixRMaj E, DMatrixRMaj Q, DMatrixRMaj R, DMatrixRMaj S);

   /**
    * Setter of the solver. A and B should be compatible. B and R must be
    * multiplicative compatible. A and Q must be multiplicative compatible. R
    * must be invertible.
    *
    * @param A state transition matrix
    * @param B control multipliers matrix
    * @param Q state cost matrix
    * @param R control cost matrix
    */
   void setMatrices(DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj E, DMatrixRMaj Q, DMatrixRMaj R);

   void setMatrices(DMatrixRMaj A, DMatrixRMaj E, DMatrixRMaj M, DMatrixRMaj Q);

   void setFinalBoundaryCondition(double finalTime, DMatrixRMaj PFinal);

   /**
    * Computes and returns the P matrix.
    */
   void computePFunction(double initialTime);

   /**
    * Returns the P matrix.
    */
   DMatrixRMaj getP(double time);
}
