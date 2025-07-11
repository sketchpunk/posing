
import splineSolver     from './splineSolver.js';

export default function splineCompose( target, chain, pose, opts={} ){
    const props = Object.assign( {
        useReach    : false,    // Allow use of Fwd & Back to try to reach target when applicable
        useLastLook : false,    // Last joint points at Target when chain is shorter than curve
    }, opts );

    // Resolve the target to the current pose data
    target.resolveTarget( chain, pose );

    // // If the target is to far away, straighten the chain
    // if( target.dist >= chain.len ){
    //     // Align the the root bone to the target direction
    //     // NOTE: Recasting since SplineTarget has all the data lookSolver needs to run, removes type error
    //     lookSolver( target as unknown as IKTarget, chain, pose );
    //     chain.resetPoseLocal( pose, 1 );
    // }else{
        return splineSolver( target, chain, pose, props );
    // }
}