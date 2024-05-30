package org.firstinspires.ftc.teamcode.purepursuit;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;

import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

public class PathSegment {
    private final HeadingPolicy policy;
    private final Vector2d start, end;
    /*
     * This variable contains the heading interpolation of a segment that utilizes the linear heading policy.
     * The interpolation works by normalizing the segment to a [0, 1] timestamp; This number represents the heading when a certain portion of the segment has been completed.
     * Notes:
     *    - Due to the nature of the normalization, the heading at timestamp 0 and 1 is the starting and ending heading, respectively.
     *    - In order to obtain the timestamp, a point on the segment *MUST* be used.
     *    - The coordinates of the point at the timestamp t use the following formula:
     *          f(t) = start * t + (1 - t) * end
     */
    private PolynomialSplineFunction interpolation;
    public PathSegment(Vector2d start, Vector2d end, HeadingPolicy policy) {
        assert policy != HeadingPolicy.LINEAR :
                "Vector2d segments don't contain heading interpolation";

        this.start = start;
        this.end = end;
        this.policy = policy;
    }

    public PathSegment(Pose2d startPose, Pose2d endPose, HeadingPolicy policy) {
        this.start = startPose.vec();
        this.end = endPose.vec();
        this.policy = policy;

        if (policy == HeadingPolicy.LINEAR)
            interpolation = new LinearInterpolator().interpolate(new double[]{0, 1}, new double[]{
                    Angle.norm(startPose.getHeading()),
                    Angle.norm(endPose.getHeading())
            });
    }

    public enum HeadingPolicy {
        /**
         * <p>This turning method disregards which side the robot is supposed to be facing.</p>
         * <p>Instead, it focuses on turning towards its target point in order to reach it as fast as possible.</p>
         * <p>Notes: Heading consistency is not guaranteed.</p>
         */
        FASTEST,
        /**
         * This method interpolates the heading linearly between the two end points of the segment.
         */
        LINEAR,
        /**
         * <p>This method maintains the robot's heading throughout the duration of the segment.</p>
         * <p>Notes: If poses are passed in, the starting heading will be maintained.</p>
         */
        CONSTANT
    }
}
