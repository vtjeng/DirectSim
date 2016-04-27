# DirectSim

## `dubins-car`
The `dubins-car` branch provides a cleaner experience of the original `DirectSim` repository. A summary of key differences
  + Options are [specified using the `bunch` library](https://github.com/vtjeng/DirectSim/commit/544e3d7b), which allows specifying parameters with dots (e.g. `options.runTime.defaultControllerTime`) while maintaining compatibility with a dictionary based-approach. This has the advantages of code completion and some checking within most modern IDEs. **NB:** This is likely to be new to users, so you should take a look at the example in `runSimple.py` to see how to work with `bunch`.
  + Cleaned up the way you specify controllers: [explicitly passing in sensor readings](https://github.com/vtjeng/DirectSim/commit/10448a91), [using an abstract base class](https://github.com/vtjeng/DirectSim/commit/44dc24d0), [controller is specified as an option of the simulation](https://github.com/vtjeng/DirectSim/commit/85828b6c). [more controllers](https://github.com/vtjeng/DirectSim/commit/f70417b3) .
  + [Added logging](https://github.com/vtjeng/DirectSim/commit/85828b6c)
  + [Zooms directly to vehicle at the start of the simulation](https://github.com/vtjeng/DirectSim/commit/6bbf13b3)
  + Removed code that was related to SARSA / machine learning work: [1](https://github.com/vtjeng/DirectSim/commit/0bf83e13), [2](https://github.com/vtjeng/DirectSim/commit/f32194aa).
  + Removed unused functions (one example [`simulate`](https://github.com/vtjeng/DirectSim/commit/205fad2c))
  + [Added a camera control panel](https://github.com/vtjeng/DirectSim/commit/a871b4a8)
  + Added cool models: 
    + [F-15](https://github.com/vtjeng/DirectSim/commit/a730ab84)
    + [X-Wing](https://github.com/vtjeng/DirectSim/commit/3668ceb0)
  
See [diff](https://github.com/vtjeng/DirectSim/compare/master...vtjeng:dubins-car) for full changes.
