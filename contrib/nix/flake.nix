{
  description = "A Nix-flake-based Rust development environment";

  inputs = {
    nixpkgs.url = "https://flakehub.com/f/NixOS/nixpkgs/0.1"; # unstable Nixpkgs
    fenix = {
      url = "https://flakehub.com/f/nix-community/fenix/0.1";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    # Use upstream sftool flake instead of a local derivation
    sftool = {
      url = "github:OpenSiFli/sftool";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs =
    { self, ... }@inputs:

    let
      systems = [
        "x86_64-linux"
        "aarch64-linux"
        "x86_64-darwin"
        "aarch64-darwin"
      ];
      forAllSystems = inputs.nixpkgs.lib.genAttrs systems;

      mkPkgs = system:
        import inputs.nixpkgs {
          inherit system;
          overlays = [ self.overlays.default ];
        };

      sftoolPkgFor = system:
        inputs.sftool.packages.${system}.sftool;
    in
    {
      overlays.default = final: prev: {
        rustToolchain =
          with inputs.fenix.packages.${prev.stdenv.hostPlatform.system};
          combine (
            with stable;
            [
              clippy
              rustc
              cargo
              # rustfmt
              rust-src
              targets."thumbv8m.main-none-eabihf".stable.rust-std
              llvm-tools
            ]
          );
      };

      packages = forAllSystems (system: {
        sftool = sftoolPkgFor system;
      });

      devShells = forAllSystems (system:
        let
          pkgs = mkPkgs system;
          sftoolPkg = sftoolPkgFor system;
        in
        {
          default = pkgs.mkShellNoCC {
            packages = (with pkgs; [
              rustToolchain
              openssl
              pkg-config
              cargo-deny
              cargo-edit
              cargo-watch
              rust-analyzer
              cargo-binutils
              probe-rs-tools
            ]) ++ [ sftoolPkg ];

            env = {
              # Required by rust-analyzer
              RUST_SRC_PATH = "${pkgs.rustToolchain}/lib/rustlib/src/rust/library";
            };
          };
        }
      );
    };
}
