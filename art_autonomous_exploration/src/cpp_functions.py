#!/usr/bin/env python

from cffi import FFI

ffi = FFI()

ffi.cdef("void brushfireFromObstacles(int ** input, int ** output, int width, int height, int min_x, int max_x, int min_y, int max_y);")
ffi.cdef("void thinning(int ** input, int ** output, int width, int height, int min_x, int max_x, int min_y, int max_y);")
ffi.cdef("void prune(int ** input, int ** output, int width, int height, int min_x, int max_x, int min_y, int max_y, int iterations);")

ffi.set_source("_cpp_functions",
    """
        #include <stdio.h>

        static void brushfireFromObstacles(int ** input, int ** output, int width, int height,
            int min_x, int max_x, int min_y, int max_y)
        {
            int i = 0;
            int j = 0;
            int step = 0;
            char changed = 1;
            while(changed == 1)
            {
                changed = 0;
                for(i = min_x ; i < max_x - 1 ; i = i + 1)
                {
                    for(j = min_y ; j < max_y - 1 ; j = j + 1)
                    {
                        if(output[i][j] == -1 && input[i][j] < 49) // Free space
                        {
                            if(
                                output[i - 1][j] == step ||
                                output[i + 1][j] == step ||
                                output[i - 1][j - 1] == step ||
                                output[i + 1][j - 1] == step ||
                                output[i - 1][j + 1] == step ||
                                output[i + 1][j + 1] == step ||
                                output[i][j - 1] == step ||
                                output[i][j + 1] == step
                            )
                            {
                                output[i][j] = step + 1;
                                changed = 1;
                            }
                        }
                    }
                }
                step = step + 1;
            }
        }

        static void thinning(int ** in, int ** out, int width, int height, int min_x, int max_x, int min_y, int max_y)
        {
            int i, j, steps = 0;
            char changed = 1;

            while(changed == 1)
            {
                steps = steps + 1;
                changed = 0;

                for(i = min_x ; i < max_x ; i = i + 1)
                    for(j = min_y ; j < max_y ; j = j + 1)
                        in[i][j] = out[i][j];

                // 000
                // x1x
                // 111
                for(i = min_x ; i < max_x - 1  ; i = i + 1)
                {
                    for(j = min_y ; j < max_y - 1 ; j = j + 1)
                    {
                        if(
                            in[i-1][j-1]==0 && in[i][j-1]==0 && in[i+1][j-1]==0
                                            && in[i][j]==1                      &&
                            in[i-1][j+1]==1 && in[i][j+1]==1 && in[i+1][j+1]==1
                        )
                        {
                            out[i][j] = 0;
                            changed = 1;
                        }
                    }
                }
                for(i = min_x ; i < max_x ; i = i + 1)
                    for(j = min_y ; j < max_y ; j = j + 1)
                        in[i][j] = out[i][j];

                // x00
                // 110
                // x1x
                for(i = min_x ; i < max_x - 1 ; i = i + 1)
                {
                    for(j = min_y ; j < max_y - 1 ; j = j + 1)
                    {
                        if(
                            1               && in[i][j-1]==0    && in[i+1][j-1]==0 &&
                            in[i-1][j]==1   && in[i][j]==1      && in[i+1][j]==0  &&
                            1               && in[i][j+1]==1    && 1
                        )
                        {
                            out[i][j] = 0;
                            changed = 1;
                        }
                    }
                }
                for(i = min_x ; i < max_x ; i = i + 1)
                    for(j = min_y ; j < max_y ; j = j + 1)
                        in[i][j] = out[i][j];

                // 1x0
                // 110
                // 1-0
                for(i = min_x ; i < max_x -1 ; i = i + 1)
                {
                    for(j = min_y ; j < max_y-1 ; j = j + 1)
                    {
                        if(
                            in[i-1][j-1]==1 && 1            && in[i+1][j-1]==0 &&
                            in[i-1][j]==1   && in[i][j]==1  && in[i+1][j]==0  &&
                            in[i-1][j+1]==1 && 1            && in[i+1][j+1]==0
                        )
                        {
                            out[i][j] = 0;
                            changed = 1;
                        }
                    }
                }
                for(i = min_x ; i < max_x ; i = i + 1)
                    for(j = min_y ; j < max_y ; j = j + 1)
                        in[i][j] = out[i][j];

                // x1x
                // 110
                // -00
                for(i = min_x ; i < max_x-1 ; i = i + 1)
                {
                    for(j = min_y ; j < max_y-1 ; j = j + 1)
                    {
                        if(
                            1               && in[i][j-1]==1    && 1                &&
                            in[i-1][j]==1   && in[i][j]==1      && in[i+1][j]==0    &&
                            1               && in[i][j+1]==0    && in[i+1][j+1]==0
                        )
                        {
                            out[i][j] = 0;
                            changed = 1;
                        }
                    }
                }
                for(i = min_x ; i < max_x ; i = i + 1)
                    for(j = min_y ; j < max_y ; j = j + 1)
                        in[i][j] = out[i][j];

                // 111
                // -1-
                // 000
                for(i = min_x ; i < max_x-1 ; i = i + 1)
                {
                    for(j = min_y ; j < max_y-1 ; j = j + 1)
                    {
                        if(
                            in[i-1][j-1]==1 && in[i][j-1]==1    && in[i+1][j-1]==1  &&
                            1               && in[i][j]==1      && 1                &&
                            in[i-1][j+1]==0 && in[i][j+1]==0    && in[i+1][j+1]==0
                        )
                        {
                            out[i][j] = 0;
                            changed = 1;
                        }
                    }
                }
                for(i = min_x ; i < max_x ; i = i + 1)
                    for(j = min_y ; j < max_y ; j = j + 1)
                        in[i][j] = out[i][j];

                // -1-
                // 011
                // 00-
                for(i = min_x ; i < max_x-1 ; i = i + 1)
                {
                    for(j = min_y ; j < max_y-1 ; j = j + 1)
                    {
                        if(
                            1               && in[i][j-1]==1    && 1                &&
                            in[i-1][j]==0   && in[i][j]==1      && in[i+1][j]==1    &&
                            in[i-1][j+1]==0 && in[i][j+1]==0    && 1
                        )
                        {
                            out[i][j] = 0;
                            changed = 1;
                        }
                    }
                }
                for(i = min_x ; i < max_y ; i = i + 1)
                    for(j = min_y ; j < max_y ; j = j + 1)
                        in[i][j] = out[i][j];

                // 0-1
                // 011
                // 0-1
                for(i = min_x ; i < max_x-1 ; i = i + 1)
                {
                    for(j = min_y ; j < max_y-1 ; j = j + 1)
                    {
                        if(
                            in[i-1][j-1]==0 && 1                && in[i+1][j-1]==1  &&
                            in[i-1][j]==0   && in[i][j]==1      && in[i+1][j]==1    &&
                            in[i-1][j+1]==0 && 1                && in[i+1][j+1]==1
                        )
                        {
                            out[i][j] = 0;
                            changed = 1;
                        }
                    }
                }
                for(i = min_x ; i < max_x ; i = i + 1)
                    for(j = min_y ; j < max_y ; j = j + 1)
                        in[i][j] = out[i][j];

                // 00-
                // 011
                // -1-
                for(i = min_x ; i < max_x-1 ; i = i + 1)
                {
                    for(j = min_y ; j < max_y-1 ; j = j + 1)
                    {
                        if(
                            in[i-1][j-1]==0 && in[i][j-1]==0    && 1  &&
                            in[i-1][j]==0   && in[i][j]==1      && in[i+1][j]==1    &&
                            1               && in[i][j+1]==1    && 1
                        )
                        {
                            out[i][j] = 0;
                            changed = 1;
                        }
                    }
                }
            } // END WHILE
        }

        static void prune(int ** in, int ** out, int width, int height, int min_x, int max_x, int min_y, int max_y, int iterations)
        {
            int i, j, steps = 0;
            int sum;

            for(steps = 0 ; steps < iterations ; steps = steps + 1)
            {
                for(i = min_x ; i < max_x ; i = i + 1)
                    for(j = min_y ; j < max_y ; j = j + 1)
                        in[i][j] = out[i][j];

                for(i = min_x ; i < max_x - 1  ; i = i + 1)
                {
                    for(j = min_y ; j < max_y - 1 ; j = j + 1)
                    {
                        sum = 
                            in[i-1][j-1] + in[i-1][j] + in[i-1][j+1] +    
                            in[i][j-1] + in[i][j] + in[i][j+1] +    
                            in[i+1][j-1] + in[i+1][j] + in[i+1][j+1];
                        if(sum == 2)
                        {
                            out[i][j] = 0;
                        }
                    }
                }
            }
        }

    """)

ffi.compile(verbose=False)
