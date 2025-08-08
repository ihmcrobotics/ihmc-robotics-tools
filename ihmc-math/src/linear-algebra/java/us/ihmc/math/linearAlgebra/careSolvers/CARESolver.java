package us.ihmc.math.linearAlgebra.careSolvers;

import org.ejml.data.DMatrixRMaj;

/**
 * This solver computes the solution to the algebraic Riccati equation
 * <p>
 * A' P + P A - P B R^-1 B' P + Q = 0
 * <p>
 * or
 * <p>
 *  A' P + P A - P M P + Q = 0
 * <p>
 * Or the more complex one
 * <p>
 * A' P E + E' P A - (B' P E + S' C)' R^-1 (B' P E + S' C) + C' Q C = 0
 * <p>
 *    or
 * <p>
 * A*' P E + E' P A* - E' P M P E + Q* = 0
 * <p>
 * where
 * <p>
 * A* = A - B R^-1 S' C
 * <p>
 * Q* = C' Q C - C' S R^-1 S' C
 * <p>
 *  I.e., we're solving the system
 *  <p>
 * J = y' Q y + u' R u' + u' S' y + y' S u
 * <p>
 * E x\u0307 = A x + B u
 * <p>
 * y = C x
 * <p>
 * It can be seen that, in the simpler system above, S = 0, E = C = I
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
 */
public interface CARESolver
{
   /**
    * Setter of the solver.
    *
    * <p>
    *    Assumes the dynamics are of the form:
    * </p>
    * <p>
    *    E x\u0307 = A x + B u
    * </p>
    * <p>
    *    y = C x
    * </p>
    * <p>
    *    And the cost function in the integrand is of the form
    * </p>
    * <p>
    * J = y' Q y + u' R u' + u' S' y + y' S u
    * </p>
    *
    * A and B should be compatible. B and R must be
    * multiplicative compatible. A and Q must be multiplicative compatible. R
    * must be invertible.
    *
    * @param A state transition matrix
    * @param B control multipliers matrix
    * @param C output matrix
    * @param E dynamics output matrix
    * @param Q state cost matrix
    * @param R control cost matrix
    * @param S cross cost matrix
    */
   void setMatrices(DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj C, DMatrixRMaj E, DMatrixRMaj Q, DMatrixRMaj R, DMatrixRMaj S);

   /**
    * Setter of the solver.
    *
    * <p>
    *    Assumes the dynamics are of the form:
    * </p>
    * <p>
    *    E x\u0307 = A x + B u
    * </p>
    * <p>
    *    And the cost function in the integrand is of the form
    * </p>
    * <p>
    * J = x' Q x + u' R u'
    * </p>
    *
    * A and B should be compatible. B and R must be
    * multiplicative compatible. A and Q must be multiplicative compatible. R
    * must be invertible.
    *
    * @param A state transition matrix
    * @param B control multipliers matrix
    * @param E dynamics output matrix
    * @param Q state cost matrix
    * @param R control cost matrix
    */
   void setMatrices(DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj E, DMatrixRMaj Q, DMatrixRMaj R);

   void setMatrices(DMatrixRMaj A, DMatrixRMaj E, DMatrixRMaj M, DMatrixRMaj Q);

   /**
    * Computes and returns the P matrix, which is the solution to the Algebraic Riccati equation.
    */
   DMatrixRMaj computeP();

   /**
    * Returns the P matrix, which is the solution to the Algebraic Riccati equation.
    */
   DMatrixRMaj getP();
}
