// #region IMPORTS
import {
    IKChain, IKTarget, limbCompose, BoneAxes, BoneMap,
    Vec3, 
} from './ossos.min.js';

import splineCompose    from './kinematics/splineCompose.js';
import IKSplineTarget   from './kinematics/IKSplineTarget.js';
import BezierQuadSpine  from './maths/BezierQuadSpline.js';
// #endregion

// #region IK SET
const IKSET_CONFIG = {
    'leftArm'   : { type:'limb', solver: limbCompose, target: IKTarget, axes: BoneAxes.RBD },
    'rightArm'  : { type:'limb', solver: limbCompose, target: IKTarget, axes: BoneAxes.LBU },

    'leftLeg'   : { type:'limb', solver: limbCompose, target: IKTarget, axes: BoneAxes.DFR },
    'rightLeg'  : { type:'limb', solver: limbCompose, target: IKTarget, axes: BoneAxes.DFR },
    'spine'     : { type:'spine', solver: splineCompose, target: IKSplineTarget, axes: BoneAxes.UFR, opts:{ useReach:false } },
};

class IKSet{
    chain       = new IKChain();
    opts        = {};
    target      = null;
    solver      = null;
    axesType    = -1;
    type        = 'unknown'
    constructor( conf=null ){
        if( conf ) this.fromConfig( conf );
    }

    setChain( bAry ){
        this.chain.setBones( bAry, this.axesType );
        return this
    }

    fromConfig( c ){
        this.target     = new c.target();
        this.solver     = c.solver;
        this.axesType   = c.axes;
        this.type       = c.type;
        this.opts       = c.opts;
        return this;
    }

    updatePose( pose ){
        return this.solver( this.target, this.chain, pose, this.opts );
    }
}
// #endregion


export class IKPoseRig{
    // #region MAIN
    tpose       = null;         // T-Pose for running IK over
    wpose       = null;         // Working Pose, final output pose

    leftLeg     = new IKSet( IKSET_CONFIG.leftLeg );
    rightLeg    = new IKSet( IKSET_CONFIG.rightLeg );
    leftArm     = new IKSet( IKSET_CONFIG.leftArm );
    rightArm    = new IKSet( IKSET_CONFIG.rightArm );
    spine       = new IKSet( IKSET_CONFIG.spine );

    spineData   = null; // Data to visualize curve & target points
    
    constructor( tpose, autoGen=false ){
        this.tpose = tpose;
        this.wpose = tpose.clone();
        if( autoGen ) this.autoGen();
    }

    autoGen(){
        const bMap = new BoneMap( this.tpose );

        this.leftArm.setChain( bMap.getBoneSet( 'leftArm' ) );
        this.leftArm.target.setPositions( this.getTargetPos( 'leftArm' ), this.getPolePos( 'leftArm' ) );

        this.rightArm.setChain( bMap.getBoneSet( 'rightArm' ) );
        this.rightArm.target.setPositions( this.getTargetPos( 'rightArm' ), this.getPolePos( 'rightArm' ) );

        this.leftLeg.setChain( bMap.getBoneSet( 'leftLeg' ) );
        this.leftLeg.target.setPositions( this.getTargetPos( 'leftLeg' ), this.getPolePos( 'leftLeg' ) );

        this.rightLeg.setChain( bMap.getBoneSet( 'rightLeg' ) );
        this.rightLeg.target.setPositions( this.getTargetPos( 'rightLeg' ), this.getPolePos( 'rightLeg' ) );

        this.spine.setChain( bMap.getBoneSet( 'spine' ) );
        this.spine.chain.usePlacementForSwing( this.tpose, [0,0,1] );
        this.spine.target.setSpline( new SpineBezierQuad() );
        this.spine.target.setPositions( this.getTargetPos( 'spine' ), this.getPolePos('spine') );

        return this;
    }
    // #endregion

    // #region GETTERS / SETTERS
    setTargetPos( sName, pos ){
        const s = this[ sName ];
        s.target.setPositions( pos );
        return this;
    }

    setPolePos( sName, pos ){
        const s = this[ sName ];
        s.target.setPolePos( pos );
        return this;
    }

    getTargetPos( sName ){
        const s = this[ sName ];
        const i = s.chain.links.at(-1).index;
        return this.wpose.getWorldTransform( i ).pos;
    }

    getPolePos( sName ){
        const s     = this[ sName ];
        const pos   = new Vec3();

        const al    = s.chain.links[0];
        const a     = this.wpose.getWorldTransform( al.index );
        const bl    = s.chain.links.at(-1);
        const b     = this.wpose.getWorldTransform( bl.index );

        switch( s.type ){
            case 'limb': 
                const dir = new Vec3().fromQuat( a.rot, al.axes.twist ).scale( 0.3 );
                pos.fromLerp( a.pos, b.pos, 0.5 ).add( dir );
                break;

            case 'spine':
                pos.fromLerp( a.pos, b.pos, 0.5 )

                const pnts = new Array( s.chain.links.length );
                for( const [i,lnk] of s.chain.links.entries() ){
                    pnts[i] = this.wpose.bones[ lnk.index ].world.pos;
                }

                const centParam = calcCentripetalParams( pnts );
                const aa = findOptimalControlPoint( pnts, centParam );
                return aa;
        }

        return pos;
    }
    // #endregion

    runSolvers(){
        this.wpose.reset();
        this.spineData = this.spine.updatePose( this.wpose );
        this.leftArm.updatePose( this.wpose );
        this.rightArm.updatePose( this.wpose );
        this.leftLeg.updatePose( this.wpose );
        this.rightLeg.updatePose( this.wpose );

        this.wpose.updateWorld();
    }
}

// #region SPINE IK SUPPORT

class SpineBezierQuad extends BezierQuadSpine{
    constructor(){
        super();
        this.appendCurve( [[0,0,0],[0,0,0],[0,0,0]] );
    }

    get rootPos(){ return this.points[0].pos.slice() };
    set rootPos( v ){ this.points[0].pos.copy(v) };
    
    get polePos(){ return this.points[1].pos.slice() };
    set polePos( v ){ this.points[1].pos.copy(v) };

    get targetPos(){ return this.points[2].pos.slice() };
    set targetPos( v ){ this.points[2].pos.copy(v) };
    resolve(){}
}


// Bezier Quad Curve Fitting - Centripetal Parameterization
function calcCentripetalParams( pnts ){
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    const eIdx = pnts.length - 1;
    if( eIdx < 1 )   return [0];
    if( eIdx === 1 ) return [0, 1];

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    const aryDist = new Array( eIdx+1 );
    let sum       = 0;
    aryDist[0]    = 0;

    for( let i=1; i <= eIdx; i++ ){
        const a = pnts[ i ];
        const b = pnts[i-1];
        
        // double sqrt is intentional: sqrt( sqrt( dist² ) ) = distance^0.5
        const lenSq = (a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2;
        const cent  = Math.sqrt( Math.sqrt( lenSq ) );
        aryDist[i]  = cent;
        sum        += cent;
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Create cumulative t-values and normalize
    const out = new Array( eIdx+1 );
    let curr  = 0;
    let t;

    out[ eIdx ] = 1;
    for( let i = 0; i < eIdx; i++ ){
        curr  += aryDist[i];
        out[i] = curr / sum;
    }

    return out;
}

// Bezier Quad Curve Fitting
function findOptimalControlPoint( pnts, wgts ) {    
    let P0 = pnts[0];               // Start point (fixed)
    let P2 = pnts[pnts.length - 1]; // End point (fixed)
    
    // We need to solve for P1 (control point) that minimizes:
    // Sum of squared distances from curve to all points
    
    // Quadratic Bézier formula: B(t) = (1-t)²P0 + 2t(1-t)P1 + t²P2
    // Rearranging: B(t) = [(1-t)²P0 + t²P2] + [2t(1-t)]P1
    // Let A(t) = (1-t)²P0 + t²P2  (known part)
    // Let w(t) = 2t(1-t)          (weight for P1)
    // So: B(t) = A(t) + w(t)*P1
    
    // For least squares, we want to minimize:
    // Σ |B(ti) - Pi|² = Σ |A(ti) + w(ti)*P1 - Pi|²
    
    let sumWeights = 0;          // Σ w(ti)²
    let sumWeightedDiff = new Vec3(); // Σ w(ti) * (Pi - A(ti))
    const A = new Vec3();
    const B = new Vec3();
    const C = new Vec3();
    const diff = new Vec3();
    
    // Skip first and last points since they're exactly on the curve
    for (let i = 1; i < pnts.length - 1; i++) {
        let t = wgts[i];
        let p = pnts[i];
        
        // Calculate weight: w(t) = 2t(1-t)
        let weight = 2 * t * (1 - t);
        
        // Calculate known part: A(t) = (1-t)²P0 + t²P2
        let ti = 1 - t;
        // let A = ti * ti * P0 + t * t *P2;  // Vector math
        A.fromScale( P0, ti*ti );
        B.fromScale( P2, t * t );
        C.fromAdd( A, B );
        
        // Calculate difference: Pi - A(ti)
        // let diff = point - A;  // [dx, dy, dz]
        diff.fromSub( p, C );
        
        // Accumulate for normal equations
        sumWeights += weight * weight;
        // sumWeightedDiff += weight * diff;  // Element-wise: [wx*dx, wy*dy, wz*dz]

        diff.scale( weight );
        sumWeightedDiff.add( diff );
    }
    
    // Solve normal equation: P1 = sumWeightedDiff / sumWeights
    // This gives us the least squares solution
    // let optimalP1 = sumWeightedDiff / sumWeights;  // Element-wise division
    A.fromScale( sumWeightedDiff, 1/sumWeights );
    
    return A;
}

// #endregion