package com.graphhopper.trailmap.barrier;

import com.graphhopper.util.exceptions.DetailedIllegalArgumentException;

import java.util.Map;

/**
 * The structured refusal of a request with {@code barriers=v1}: a route point can only be reached through a barrier
 * of class REFUSE. GraphHopper's error serializer writes {@link #getDetails()} into the error hint, next to
 * {@code details} (this class's name), so the client gets the barrier, the reason code and the nearest approach.
 */
public class BarrierRefusedException extends DetailedIllegalArgumentException {

    private static final long serialVersionUID = 1L;

    public BarrierRefusedException(String message, Map<String, Object> details) {
        super(message, details);
    }
}
